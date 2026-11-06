"""No-op stand-ins for the drone-show helpers that this fork removed
(scheduled takeoff, LED light config, time axis config). mavlink/network.py
and mavlink/extension.py still construct and call them; here they do nothing.
"""

from contextlib import contextmanager
from typing import Any

from trio import sleep_forever


class _NoOpManager:
    """Stand-in for the per-network managers."""

    _log: Any = None

    def __init__(self, network: Any = None) -> None:
        self._network = network

    async def run(self) -> None:
        await sleep_forever()

    def notify_config_changed(self, *args: Any, **kwds: Any) -> None:
        pass

    def notify_start_time_changed(self, *args: Any, **kwds: Any) -> None:
        pass

    def notify_time_axis_config_changed(self, *args: Any, **kwds: Any) -> None:
        pass


class _NoOpDispatcher:
    """Stand-in for the extension-level signal dispatchers."""

    @contextmanager
    def use(self, signals: Any, log: Any = None):
        yield self

    async def run(self) -> None:
        await sleep_forever()

    def notify_config_changed(self, *args: Any, **kwds: Any) -> None:
        pass

    def notify_start_time_changed(self, *args: Any, **kwds: Any) -> None:
        pass

    def notify_time_axis_config_changed(self, *args: Any, **kwds: Any) -> None:
        pass


class MAVLinkLEDLightConfigurationManager(_NoOpManager):
    pass


class MAVLinkScheduledTakeoffManager(_NoOpManager):
    pass


class MAVLinkTimeAxisConfigurationManager(_NoOpManager):
    pass


class LEDLightConfigurationSignalDispatcher(_NoOpDispatcher):
    pass


class ScheduledTakeoffSignalDispatcher(_NoOpDispatcher):
    pass


class TimeAxisConfigurationSignalDispatcher(_NoOpDispatcher):
    pass
