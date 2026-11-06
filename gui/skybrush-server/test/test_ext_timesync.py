from typing import Any

from pytest import approx

from flockwave.server.ext.timesync.manager import TimeSyncManager


class MockClock:
    def __init__(self, value: float = 0.0):
        self.value = value

    def __call__(self) -> float:
        return self.value


class MockLogger:
    def __init__(self):
        self.messages: list[tuple[str, str, dict[str, Any] | None]] = []

    def info(self, msg: str, *, extra: dict[str, Any] | None = None) -> None:
        self.messages.append(("info", msg, extra))

    def warning(self, msg: str, *, extra: dict[str, Any] | None = None) -> None:
        self.messages.append(("warning", msg, extra))


def test_initial_state_is_unknown() -> None:
    wall_clock = MockClock(100.0)
    monotonic_clock = MockClock(50.0)
    manager = TimeSyncManager(
        current_time=wall_clock,
        current_monotonic_time=monotonic_clock,
    )

    status = manager.get_status()

    assert status.state == "unknown"
    assert status.offset is None
    assert status.selected_source is None
    assert not status.in_sync


def test_single_source_with_small_offset_is_sync() -> None:
    wall_clock = MockClock(100.0)
    monotonic_clock = MockClock(10.0)
    logger = MockLogger()
    manager = TimeSyncManager(
        current_time=wall_clock,
        current_monotonic_time=monotonic_clock,
        log=logger,
    )

    source = manager.register_time_source("ntp", priority=50)
    source.submit_timestamp(100.02, jitter=0.001)

    status = manager.get_status()

    assert status.state == "sync"
    assert status.selected_source == "ntp"
    assert status.offset == approx(0.02)
    assert status.jitter == 0.001
    assert logger.messages == [
        (
            "info",
            "Server clock is synchronized to world clock",
            {"semantics": "success"},
        )
    ]


def test_jitter_is_informational_only() -> None:
    wall_clock = MockClock(100.0)
    monotonic_clock = MockClock(10.0)
    manager = TimeSyncManager(
        current_time=wall_clock,
        current_monotonic_time=monotonic_clock,
    )

    source = manager.register_time_source("rtk", priority=100)
    source.submit_timestamp(100.2, jitter=0.5)

    status = manager.get_status()

    assert status.state == "unsync"
    assert status.offset == approx(0.2)
    assert status.jitter == 0.5


def test_source_can_submit_offset_directly() -> None:
    wall_clock = MockClock(100.0)
    monotonic_clock = MockClock(10.0)
    manager = TimeSyncManager(
        current_time=wall_clock,
        current_monotonic_time=monotonic_clock,
    )

    source = manager.register_time_source("ntp", priority=100)
    source.submit_offset(0.02, jitter=0.001)

    status = manager.get_status()
    record = manager._get_source("ntp")

    assert status.state == "sync"
    assert status.offset == approx(0.02)
    assert status.jitter == 0.001
    assert record.last_reported_offset == approx(0.02)


def test_highest_priority_non_error_source_wins() -> None:
    wall_clock = MockClock(100.0)
    monotonic_clock = MockClock(10.0)
    manager = TimeSyncManager(
        current_time=wall_clock,
        current_monotonic_time=monotonic_clock,
    )

    low = manager.register_time_source("low", priority=10)
    high = manager.register_time_source("high", priority=20)

    low.submit_timestamp(100.01)
    high.submit_timestamp(100.2)

    status = manager.get_status()

    assert status.selected_source == "high"
    assert status.state == "unsync"
    assert status.offset == approx(0.2)

    high.submit_error("no signal")
    status = manager.get_status()

    assert status.selected_source == "low"
    assert status.state == "sync"
    assert status.offset == approx(0.01)


def test_most_recent_submission_wins_on_equal_priority() -> None:
    wall_clock = MockClock(100.0)
    monotonic_clock = MockClock(10.0)
    manager = TimeSyncManager(
        current_time=wall_clock,
        current_monotonic_time=monotonic_clock,
    )

    first = manager.register_time_source("first", priority=20)
    second = manager.register_time_source("second", priority=20)

    first.submit_timestamp(100.01)
    monotonic_clock.value = 11.0
    second.submit_timestamp(100.02)

    status = manager.get_status()

    assert status.selected_source == "second"
    assert status.offset == approx(0.02)

    monotonic_clock.value = 12.0
    first.submit_timestamp(100.03)
    status = manager.get_status()

    assert status.selected_source == "first"
    assert status.offset == approx(0.03)


def test_all_reporting_sources_in_error_yield_error_state() -> None:
    wall_clock = MockClock(100.0)
    monotonic_clock = MockClock(10.0)
    logger = MockLogger()
    manager = TimeSyncManager(
        current_time=wall_clock,
        current_monotonic_time=monotonic_clock,
        log=logger,
    )

    source = manager.register_time_source("ntp", priority=10)
    source.submit_error("timeout")

    status = manager.get_status()

    assert status.state == "error"
    assert status.selected_source is None
    assert logger.messages == [
        (
            "warning",
            "Cannot determine server clock synchronization state: all reporting "
            "time sources indicated errors",
            None,
        )
    ]


def test_expired_sources_are_ignored() -> None:
    wall_clock = MockClock(100.0)
    monotonic_clock = MockClock(10.0)
    logger = MockLogger()
    manager = TimeSyncManager(
        current_time=wall_clock,
        current_monotonic_time=monotonic_clock,
        source_expiry_threshold=5.0,
        log=logger,
    )

    source = manager.register_time_source("ntp", priority=10)
    source.submit_timestamp(100.01)

    monotonic_clock.value = 16.0
    status = manager.refresh_status()

    assert status.state == "unknown"
    assert status.selected_source is None
    assert logger.messages[-1] == (
        "info",
        "Cannot determine server clock synchronization state: no recent timestamp is available",
        None,
    )


def test_offset_log_threshold_controls_repeated_messages() -> None:
    wall_clock = MockClock(100.0)
    monotonic_clock = MockClock(10.0)
    logger = MockLogger()
    manager = TimeSyncManager(
        current_time=wall_clock,
        current_monotonic_time=monotonic_clock,
        sync_threshold=0.5,
        offset_log_threshold=0.05,
        log=logger,
    )

    source = manager.register_time_source("ntp", priority=10)
    source.submit_timestamp(100.10)
    source.submit_timestamp(100.12)
    source.submit_timestamp(100.18)

    assert len(logger.messages) == 2
    assert logger.messages[0][0] == "info"
    assert logger.messages[1] == (
        "info",
        "Server clock is synchronized to world clock",
        {"semantics": "success"},
    )


def test_context_manager_unregisters_source() -> None:
    wall_clock = MockClock(100.0)
    monotonic_clock = MockClock(10.0)
    manager = TimeSyncManager(
        current_time=wall_clock,
        current_monotonic_time=monotonic_clock,
    )

    with manager.use_time_source("ntp", priority=10) as source:
        source.submit_timestamp(100.01)
        assert manager.get_status().state == "sync"

    status = manager.get_status()

    assert status.state == "unknown"
