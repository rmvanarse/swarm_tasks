from typing import Any, Literal, Protocol

__all__ = ("LoggerLike", "TimeSyncState")


TimeSyncState = Literal["sync", "unsync", "unknown", "error"]


class LoggerLike(Protocol):
    """Minimal logger interface used by the time synchronization manager."""

    def info(self, msg: str, *, extra: dict[str, Any] | None = None) -> Any: ...
    def warning(self, msg: str, *, extra: dict[str, Any] | None = None) -> Any: ...
