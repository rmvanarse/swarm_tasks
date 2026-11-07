"""Warns when a swarm command would take drones too close to the ground.

After a goal, search, split or specific split has been accepted, the route or
area is checked against the terrain data, at the height each drone is set to fly
at. This only warns; the command is never held back. Without terrain data, or
without knowing a drone's height, nothing is said.

Heights set on the swarm side are metres above home; the terrain data is metres
above sea level, so each drone's home altitude (its altitude above sea level
minus its altitude above home) is used to convert.
"""

from __future__ import annotations

from typing import Any, Awaitable, Callable, Mapping

__all__ = ("normalise_id", "terrain_warning")

NAMES = {
    "goal": "Goal",
    "search": "Search",
    "split": "Split",
    "specificsplit": "Specific split",
}


def normalise_id(value: Any) -> str:
    """"03" and 3 are the same drone."""
    text = str(value).strip()
    return str(int(text)) if text.isdigit() else text


def _pair(point) -> tuple[float, float] | None:
    try:
        return float(point[0]), float(point[1])
    except (TypeError, ValueError, IndexError):
        return None


async def terrain_warning(
    command: str,
    uav_ids: list[str],
    kwds: Mapping[str, Any],
    *,
    drones: Mapping[str, tuple[float, float, float]],
    heights: Mapping[str, float],
    terrain: Any,
    min_clearance_m: float,
) -> str | None:
    """Returns a warning for the command, or None if all is well or unknown.

    Args:
        uav_ids: the drones the command is for, as selected in Live
        drones: id -> (lat, lon, home altitude above sea level) for known drones
        heights: id -> height above home the swarm will fly the drone at
        terrain: object with async check_route_clearance(points, altitude_amsl)
            and check_area_clearance(centers, size_m, altitude_amsl)
    """
    if command not in NAMES:
        return None

    targets = _targets(command, uav_ids, kwds, drones)
    if not targets:
        return None

    low: list[tuple[str, float]] = []
    worst_ground = None

    for drone_id in targets:
        height = heights.get(drone_id)
        known = drones.get(drone_id)
        if height is None or known is None:
            continue
        lat, lon, home_amsl = known
        altitude = home_amsl + float(height)

        if command == "goal":
            waypoints = [p for p in map(_pair, kwds.get("goal_latlon") or []) if p]
            if not waypoints:
                return None
            result = await terrain.check_route_clearance([(lat, lon)] + waypoints, altitude)
        else:
            centers, size = _area(command, kwds)
            if not centers:
                return None
            result = await terrain.check_area_clearance(centers, size, altitude)

        if result is None:
            continue
        if worst_ground is None or result["ground_max"] > worst_ground:
            worst_ground = result["ground_max"]
        if result["min_clearance"] < min_clearance_m:
            low.append((drone_id, result["min_clearance"]))

    if not low:
        return None

    what = "route" if command == "goal" else "area"
    drones_text = ", ".join(f"UAV {i} ({c:+.0f} m)" for i, c in low)
    return (
        f"{NAMES[command]}: the ground on this {what} rises to "
        f"{worst_ground:.0f} m above sea level. At their set heights these drones "
        f"would be below the {min_clearance_m:.0f} m minimum clearance: "
        f"{drones_text}. The command was sent anyway."
    )


def _targets(command, uav_ids, kwds, drones) -> list[str]:
    if command == "goal":
        raw = kwds.get("ids") if kwds.get("ids") is not None else uav_ids
    elif command == "search":
        raw = list(drones)  # search uses every drone
    elif command == "split":
        raw = (
            kwds.get("selected_uav_ids")
            if kwds.get("selected_uav_ids") is not None
            else uav_ids
        )
    else:  # specificsplit: groups of ids
        raw = [i for group in (kwds.get("uav_array") or []) for i in group]
    return list(dict.fromkeys(normalise_id(i) for i in raw or []))


def _area(command, kwds) -> tuple[list[tuple[float, float]], float]:
    if command == "search":
        centre = _pair((kwds.get("center_lat"), kwds.get("center_lon")))
        centers = [centre] if centre else []
    else:
        centers = [p for p in map(_pair, kwds.get("center_lat_lon_array") or []) if p]
    try:
        size = float(kwds.get("coverage_area"))
    except (TypeError, ValueError):
        size = 0.0
    return (centers, size) if size > 0 else ([], 0.0)
