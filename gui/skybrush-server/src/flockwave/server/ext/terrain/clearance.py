"""Height-above-ground checks and the warnings built on them.

All heights are metres above mean sea level unless stated otherwise; the
"clearance" of a point is its altitude minus the ground height below it.
"""

from __future__ import annotations

import math
import time
from logging import Logger
from typing import Any, Callable, Iterable

import numpy as np

__all__ = (
    "ClearanceMonitor",
    "METRES_PER_DEGREE",
    "area_points",
    "predict_clearance",
    "summarize_clearance",
)

METRES_PER_DEGREE = 111318.845

GroundFn = Callable[[Any, Any], np.ndarray]


def _lon_scale(lat: float) -> float:
    return max(math.cos(math.radians(lat)), 0.01)


def predict_clearance(
    lat: float,
    lon: float,
    amsl: float,
    vn: float,
    ve: float,
    vd: float,
    lookahead_s: float,
    ground: GroundFn,
    step_s: float = 1.0,
) -> tuple[float, float, float] | None:
    """Lowest clearance on the way ahead if the drone keeps its velocity.

    `vn`, `ve`, `vd` are the velocity components north, east and down (m/s).
    Returns (clearance, seconds from now, ground height there), or None when the
    ground height is unknown everywhere on the way.
    """
    t = np.arange(0.0, max(lookahead_s, 0.0) + 1e-9, max(step_s, 0.1))
    lats = lat + vn * t / METRES_PER_DEGREE
    lons = lon + ve * t / (METRES_PER_DEGREE * _lon_scale(lat))
    heights = ground(lats, lons)
    clearance = (amsl - vd * t) - heights
    if not np.isfinite(clearance).any():
        return None
    i = int(np.nanargmin(np.where(np.isfinite(clearance), clearance, np.inf)))
    return float(clearance[i]), float(t[i]), float(heights[i])


def summarize_clearance(lats, lons, heights, altitude_amsl: float) -> dict | None:
    """The worst point of a route flown at a constant altitude: the one where
    the ground is highest. None if no ground height is known along it."""
    heights = np.asarray(heights, dtype=float)
    known = np.isfinite(heights)
    if not known.any():
        return None
    clearance = altitude_amsl - heights
    i = int(np.nanargmin(np.where(known, clearance, np.inf)))
    return {
        "min_clearance": float(clearance[i]),
        "ground_max": float(heights[i]),
        "lat": float(np.asarray(lats)[i]),
        "lon": float(np.asarray(lons)[i]),
        "unknown": int((~known).sum()),
        "samples": int(len(heights)),
    }


def area_points(lat: float, lon: float, size_m: float, max_samples: int = 900):
    """Sample points over the square of side `size_m` centred on (lat, lon),
    at most `max_samples` of them, as (lats, lons) arrays."""
    half = max(size_m, 1.0) / 2.0
    n = int(min(max(math.isqrt(max_samples), 2), max(2, math.ceil(size_m / 30.0) + 1)))
    offsets = np.linspace(-half, half, n)
    north, east = np.meshgrid(offsets, offsets, indexing="ij")
    lats = lat + north.ravel() / METRES_PER_DEGREE
    lons = lon + east.ravel() / (METRES_PER_DEGREE * _lon_scale(lat))
    return lats, lons


class ClearanceMonitor:
    """Watches the drones' height above ground and warns when it gets low,
    now or within the look-ahead time at the current velocity.

    A warning is raised once when a drone drops below the limit, repeated every
    `REPEAT_S` seconds while it stays low, and cleared when the clearance is back
    above the limit plus `HYSTERESIS_M`, so a drone hovering around the limit does
    not flood the operator. Drones on the ground are ignored.
    """

    HYSTERESIS_M = 5.0
    REPEAT_S = 60.0
    MIN_FLYING_HEIGHT_M = 3.0

    def __init__(
        self,
        uavs: Callable[[], Iterable[tuple[str, Any]]],
        ground: GroundFn,
        notify: Callable[[str], None],
        settings: dict[str, Any],
        log: Logger,
        clock: Callable[[], float] = time.monotonic,
    ):
        self._uavs = uavs
        self._ground = ground
        self._notify = notify
        self._settings = settings
        self._log = log
        self._clock = clock
        self._warned_at: dict[str, float] = {}
        #: Drones that are currently too low: id -> details
        self.active: dict[str, dict[str, Any]] = {}
        #: Flying drones over ground we have no data for: id -> (lat, lon)
        self.unknown: dict[str, tuple[float, float]] = {}

    def step(self) -> list[str]:
        """Checks every drone once; returns the warnings that were raised."""
        limit = float(self._settings["min_clearance_m"])
        lookahead = float(self._settings["lookahead_s"])
        raised = []
        seen = set()
        self.unknown = {}

        for uav_id, status in self._uavs():
            seen.add(uav_id)
            info = self._evaluate(status, lookahead)
            if info is None:
                self._clear(uav_id)
                continue
            if "unknown_at" in info:
                self.unknown[uav_id] = info["unknown_at"]
                self._clear(uav_id)
                continue

            low = info["clearance"] < limit
            recovered = info["clearance"] > limit + self.HYSTERESIS_M
            if low:
                self.active[uav_id] = info
                last = self._warned_at.get(uav_id)
                now = self._clock()
                if last is None or now - last >= self.REPEAT_S:
                    self._warned_at[uav_id] = now
                    message = self._message(uav_id, info, limit)
                    self._notify(message)
                    raised.append(message)
            elif recovered:
                self._clear(uav_id)

        for uav_id in list(self.active):
            if uav_id not in seen:
                self._clear(uav_id)
        return raised

    def _clear(self, uav_id: str) -> None:
        self.active.pop(uav_id, None)
        self._warned_at.pop(uav_id, None)

    def _evaluate(self, status, lookahead: float) -> dict[str, Any] | None:
        position = getattr(status, "position", None)
        if position is None:
            return None
        lat, lon = position.lat, position.lon
        amsl, ahl = position.amsl, position.ahl
        if amsl is None or lat is None or lon is None or (lat == 0 and lon == 0):
            return None
        if ahl is None or ahl < self.MIN_FLYING_HEIGHT_M:
            return None  # on the ground (or just lifting off)

        velocity = getattr(status, "velocity", None)
        vn = getattr(velocity, "x", 0.0) or 0.0
        ve = getattr(velocity, "y", 0.0) or 0.0
        vd = getattr(velocity, "z", 0.0) or 0.0
        result = predict_clearance(
            lat, lon, amsl, vn, ve, vd, lookahead, self._ground
        )
        if result is None:
            return {"unknown_at": (lat, lon)}
        clearance, seconds, ground = result
        return {"clearance": clearance, "in_seconds": seconds, "ground": ground}

    @staticmethod
    def _message(uav_id: str, info: dict[str, Any], limit: float) -> str:
        when = "now" if info["in_seconds"] < 0.5 else f"in {info['in_seconds']:.0f} s"
        return (
            f"UAV {uav_id}: only {info['clearance']:.0f} m above the ground "
            f"{when} (limit {limit:.0f} m; ground there is "
            f"{info['ground']:.0f} m above sea level)"
        )
