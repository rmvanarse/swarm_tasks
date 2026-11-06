from __future__ import annotations

from dataclasses import dataclass
from typing import TypedDict

from .types import TimeSyncState

__all__ = ("TimeSyncSnapshot", "TimeSyncSnapshotDict", "TimeSourceRecord")


@dataclass(frozen=True)
class TimeSyncSnapshot:
    """Immutable snapshot describing the current synchronization status."""

    state: TimeSyncState = "unknown"
    """The current synchronization state of the server clock."""

    selected_source: str | None = None
    """Identifier of the currently selected time source, if any."""

    offset: float | None = None
    """Measured wall-clock offset in seconds.

    The value is defined as ``wall_clock_time - local_server_time``. Positive
    values mean that the local clock lags behind wall clock time; negative
    values mean that the local clock is ahead.
    """

    jitter: float | None = None
    """Reported jitter of the selected time source, in seconds, if known."""

    @property
    def in_sync(self) -> bool:
        """Returns whether the local clock is considered synchronized."""
        return self.state == "sync"

    @property
    def json(self) -> TimeSyncSnapshotDict:
        """Returns the snapshot in a JSON-serializable representation."""
        return {
            "state": self.state,
            "source": self.selected_source,
            "offset": self.offset,
            "jitter": self.jitter,
        }


class TimeSyncSnapshotDict(TypedDict):
    """JSON-serializable representation of a time synchronization snapshot."""

    state: TimeSyncState
    source: str | None
    offset: float | None
    jitter: float | None


@dataclass
class TimeSourceRecord:
    """Internal state associated with a single registered time source."""

    id: str
    """Unique identifier of the time source."""

    priority: int = 0
    """Priority of the time source; higher values take precedence."""

    last_event_at_monotonic: float | None = None
    """Time of the last report from the source, from ``time.monotonic()``.

    This value is monotonic and is suitable only for measuring elapsed time.
    It is not a UNIX timestamp and must not be interpreted as wall clock time.
    """

    last_success_at_monotonic: float | None = None
    """Time of the last successful timestamp or offset report, from
    ``time.monotonic()``.

    This value is monotonic and is used only for age and recency comparisons.
    It is not a UNIX timestamp.
    """

    last_reported_jitter: float | None = None
    """Last reported jitter in seconds, if the source supplied one."""

    last_reported_offset: float | None = None
    """Last reported or derived wall-clock offset in seconds.

    The value is defined as ``wall_clock_time - local_server_time``.
    """

    last_error: str | None = None
    """String representation of the last error reported by the source, if any."""

    @property
    def has_report(self) -> bool:
        """Returns whether the source has reported any state yet."""
        return self.last_event_at_monotonic is not None

    @property
    def is_error(self) -> bool:
        """Returns whether the latest report from the source is an error."""
        return self.last_error is not None

    def is_active(self, now_monotonic: float, expiry_threshold: float) -> bool:
        """Returns whether the latest successful report is still usable.

        Args:
            now_monotonic: current monotonic timestamp from ``time.monotonic()``
            expiry_threshold: maximum allowed age of the last successful report,
                in seconds
        """
        if (
            self.is_error
            or self.last_success_at_monotonic is None
            or self.last_reported_offset is None
        ):
            return False

        return now_monotonic - self.last_success_at_monotonic <= expiry_threshold
