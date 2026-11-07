"""Terrain extension: ground elevation for the map, the drones and warnings.

Elevation comes from SRTM tiles (see dem.py). Live asks for:

  GET  /terrain/tiles/<z>/<x>/<y>.png?style=hillshade|relief   map overlay
  GET  /terrain/elevation?lat=..&lon=..                         one point
  POST /terrain/profile  {"points": [[lat, lon], ...]}          along a path
  POST /terrain/prefetch {"lat", "lon", "radius_km"}            prepare an area
  GET  /terrain/stats                                            what is stored

Other parts of the server use the functions in ``exports``.

Heights are metres above mean sea level. Ground below sea level (open ocean)
is reported as 0, because the sea floor is not an obstacle.
"""

from __future__ import annotations

import math
from logging import Logger
from typing import Any

import numpy as np
from quart import Response, request
from trio import open_nursery, sleep, sleep_forever, to_thread

from flockwave.server.ext.http_server import HTTPServerExtensionAPI
from flockwave.server.model.log import Severity
from flockwave.server.model.uav import UAV
from flockwave.server.utils.quart import make_blueprint

from .clearance import ClearanceMonitor, area_points, summarize_clearance
from .dem import DEFAULT_SOURCE_URL, TerrainStore, tiles_in_circle
from .hillshade import MAX_ZOOM, MIN_ZOOM, STYLES, render_tile
from .schema import schema as schema  # noqa: F401 (picked up by the loader)

__all__ = ("dependencies", "description", "exports", "run", "schema")

dependencies = ("http_server",)
description = "Terrain elevation data, map overlay and height-above-ground warnings"

MAX_PROFILE_SAMPLES = 5000

#: Map tiles below this zoom are drawn only from data already on disk (a screen
#: there spans several degrees, i.e. many 26 MB SRTM files).
MIN_DOWNLOAD_ZOOM = 11
EARTH_RADIUS_M = 6371008.8

_store: TerrainStore | None = None
_monitor: ClearanceMonitor | None = None
_settings: dict[str, Any] = {"min_clearance_m": 30.0, "lookahead_s": 10.0}


# -- functions for other parts of the server ----------------------------------


def get_ground_heights(lats, lons) -> np.ndarray:
    """Ground height above sea level (m) at each point; NaN where unknown.
    Never negative. Does not download anything (it is called from the event
    loop); tiles are fetched by the tile and profile routes and by prefetch."""
    if _store is None:
        return np.full(np.shape(np.atleast_1d(lats)), np.nan)
    return np.maximum(_store.elevations(lats, lons), 0.0)


def get_ground_height(lat: float, lon: float) -> float | None:
    """Ground height above sea level (m) at one point, or None if unknown."""
    value = float(get_ground_heights([lat], [lon])[0])
    return None if math.isnan(value) else value


def sample_path(points, spacing_m: float = 30.0):
    """Points along a path, at most `spacing_m` apart, as (distance_m, lat, lon)
    arrays. `points` is a list of (lat, lon)."""
    pts = [(float(a), float(b)) for a, b in points]
    if not pts:
        return np.zeros(0), np.zeros(0), np.zeros(0)
    dist, lats, lons = [0.0], [pts[0][0]], [pts[0][1]]
    total = 0.0
    spacing_m = max(float(spacing_m), 1.0)
    for (la1, lo1), (la2, lo2) in zip(pts, pts[1:]):
        seg = _distance_m(la1, lo1, la2, lo2)
        steps = max(1, math.ceil(seg / spacing_m))
        for i in range(1, steps + 1):
            f = i / steps
            lats.append(la1 + (la2 - la1) * f)
            lons.append(lo1 + (lo2 - lo1) * f)
            dist.append(total + seg * f)
        total += seg
    return np.array(dist), np.array(lats), np.array(lons)


def get_path_profile(points, spacing_m: float = 30.0) -> dict[str, Any]:
    """Ground heights along a path (does not download)."""
    dist, lats, lons = sample_path(points, spacing_m)
    heights = get_ground_heights(lats, lons) if len(lats) else np.zeros(0)
    known = heights[np.isfinite(heights)]
    return {
        "distance": dist,
        "lat": lats,
        "lon": lons,
        "height": heights,
        "max": float(known.max()) if len(known) else None,
        "min": float(known.min()) if len(known) else None,
        "unknown": int(len(heights) - len(known)),
    }


async def fetch_ground_heights(lats, lons) -> np.ndarray:
    """Like get_ground_heights(), but downloads a missing tile first (in a
    worker thread, so the event loop keeps running). Still NaN where the data
    cannot be obtained, e.g. when offline."""
    store = _store
    if store is None:
        return np.full(np.shape(np.atleast_1d(lats)), np.nan)
    return await to_thread.run_sync(
        lambda: np.maximum(store.elevations(lats, lons, download=True), 0.0)
    )


async def check_route_clearance(points, altitude_amsl: float, spacing_m: float = 30.0):
    """The worst point of a route flown at `altitude_amsl`: where the ground is
    highest. `points` is a list of (lat, lon). Downloads missing tiles first.
    Returns None if no ground height is known along the route, otherwise a dict
    with min_clearance, ground_max, lat, lon, unknown and samples."""
    pts = [(float(a), float(b)) for a, b in points]
    if not pts:
        return None
    length = sum(
        _distance_m(a[0], a[1], b[0], b[1]) for a, b in zip(pts, pts[1:])
    )
    spacing = max(spacing_m, length / (MAX_PROFILE_SAMPLES - 1))
    _, lats, lons = sample_path(pts, spacing)
    heights = await fetch_ground_heights(lats, lons)
    return summarize_clearance(lats, lons, heights, altitude_amsl)


async def check_area_clearance(centers, size_m: float, altitude_amsl: float):
    """Like check_route_clearance(), for squares of side `size_m` centred on
    each of `centers` (a list of (lat, lon)): a search or split area."""
    lat_parts, lon_parts = [], []
    for lat, lon in centers:
        lats, lons = area_points(float(lat), float(lon), float(size_m))
        lat_parts.append(lats)
        lon_parts.append(lons)
    if not lat_parts:
        return None
    lats, lons = np.concatenate(lat_parts), np.concatenate(lon_parts)
    heights = await fetch_ground_heights(lats, lons)
    return summarize_clearance(lats, lons, heights, altitude_amsl)


def get_active_warnings() -> dict[str, dict[str, Any]]:
    """Drones that are currently too close to the ground."""
    return dict(_monitor.active) if _monitor is not None else {}


def get_settings() -> dict[str, Any]:
    return dict(_settings)


def is_ready() -> bool:
    return _store is not None


exports = {
    "get_ground_height": get_ground_height,
    "get_ground_heights": get_ground_heights,
    "fetch_ground_heights": fetch_ground_heights,
    "check_route_clearance": check_route_clearance,
    "check_area_clearance": check_area_clearance,
    "get_active_warnings": get_active_warnings,
    "get_path_profile": get_path_profile,
    "sample_path": sample_path,
    "get_settings": get_settings,
    "is_ready": is_ready,
}


def _distance_m(lat1, lon1, lat2, lon2) -> float:
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(a)))


def _number(value) -> float | None:
    return None if value is None or not math.isfinite(value) else round(float(value), 1)


# -- extension ------------------------------------------------------------------


async def run(app, configuration, logger: Logger):
    global _store, _monitor

    _settings["min_clearance_m"] = float(configuration.get("min_clearance_m", 30))
    _settings["lookahead_s"] = float(configuration.get("lookahead_s", 10))

    cache_dir = app.dirs.user_cache_path / "ext" / "terrain"
    store = TerrainStore(
        cache_dir,
        source_url=str(configuration.get("source_url") or DEFAULT_SOURCE_URL),
        offline=bool(configuration.get("offline", False)),
        logger=logger,
        data_dirs=[str(d) for d in configuration.get("data_dirs") or []],
    )
    _store = store

    blueprint = make_blueprint("terrain", __name__)

    @blueprint.after_request
    async def allow_cross_origin(response):
        # Live is served from another origin (its dev server or the Electron
        # shell), so the browser asks permission before it calls these routes,
        # including the pre-flight OPTIONS request of the POST ones.
        response.headers["Access-Control-Allow-Origin"] = "*"
        response.headers["Access-Control-Allow-Methods"] = "GET, POST, OPTIONS"
        response.headers["Access-Control-Allow-Headers"] = "Content-Type"
        return response

    @blueprint.route("/tiles/<int:z>/<int:x>/<int:y>.png")
    async def get_tile(z: int, x: int, y: int):
        style = request.args.get("style", "hillshade")
        if style not in STYLES:
            style = "hillshade"
        # Only fetch missing elevation data when zoomed in. One SRTM file is
        # ~26 MB and a screen at low zoom spans several degrees, so panning
        # around a zoomed-out map would otherwise download dozens of files.
        # Zoomed out, what is already on disk is drawn; "Download this area"
        # (or prefetch_areas) is the way to get a wider region.
        contours = request.args.get("contours", "1") != "0"
        body = await to_thread.run_sync(
            lambda: render_tile(
                store,
                z,
                x,
                y,
                style,
                download=z >= MIN_DOWNLOAD_ZOOM,
                contours=contours,
            )
        )
        return Response(
            body,
            content_type="image/png",
            headers={
                "Cache-Control": "public, max-age=3600",
                "Access-Control-Allow-Origin": "*",
            },
        )

    @blueprint.route("/elevation")
    async def get_elevation():
        try:
            lat, lon = float(request.args["lat"]), float(request.args["lon"])
        except (KeyError, ValueError):
            return {"error": "lat and lon are required numbers"}, 400
        value = await to_thread.run_sync(
            lambda: store.elevations([lat], [lon], download=True)[0]
        )
        return {
            "lat": lat,
            "lon": lon,
            "elevation": _number(max(value, 0.0) if math.isfinite(value) else None),
        }

    @blueprint.route("/profile", methods=["POST"])
    async def get_profile():
        body = await request.get_json(silent=True) or {}
        try:
            points = [(float(a), float(b)) for a, b in body.get("points", [])]
            spacing = float(body.get("spacing", 30))
        except (TypeError, ValueError):
            return {"error": "points must be [[lat, lon], ...]"}, 400
        if len(points) < 1:
            return {"error": "at least one point is required"}, 400

        dist, lats, lons = sample_path(points, spacing)
        if len(lats) > MAX_PROFILE_SAMPLES:
            return {"error": f"too many samples (limit {MAX_PROFILE_SAMPLES})"}, 400
        heights = await to_thread.run_sync(
            lambda: np.maximum(store.elevations(lats, lons, download=True), 0.0)
        )
        known = heights[np.isfinite(heights)]
        return {
            "distance": [round(float(d), 1) for d in dist],
            "lat": [round(float(v), 7) for v in lats],
            "lon": [round(float(v), 7) for v in lons],
            "height": [_number(float(h)) for h in heights],
            "max": _number(known.max()) if len(known) else None,
            "min": _number(known.min()) if len(known) else None,
            "unknown": int(len(heights) - len(known)),
        }

    @blueprint.route("/prefetch", methods=["POST"])
    async def prefetch():
        body = await request.get_json(silent=True) or {}
        try:
            lat, lon = float(body["lat"]), float(body["lon"])
            radius = min(float(body.get("radius_km", 20)), 300.0)
        except (KeyError, TypeError, ValueError):
            return {"error": "lat, lon and radius_km are required numbers"}, 400
        names = tiles_in_circle(lat, lon, radius)
        summary = await to_thread.run_sync(lambda: store.prefetch(names))
        logger.info(f"Terrain prefetch around {lat:.3f}, {lon:.3f}: {summary}")
        return {"tiles": names, "result": summary}

    @blueprint.route("/warnings")
    async def get_warnings():
        return {
            "min_clearance_m": _settings["min_clearance_m"],
            "lookahead_s": _settings["lookahead_s"],
            "low": {
                uav_id: {k: _number(v) for k, v in info.items()}
                for uav_id, info in get_active_warnings().items()
            },
        }

    @blueprint.route("/stats")
    async def get_stats():
        count, size = await to_thread.run_sync(store.stats)
        return {
            "tiles": count,
            "bytes": size,
            "names": store.available_tiles(),
            "offline": store.offline,
            "min_zoom": MIN_ZOOM,
            "max_zoom": MAX_ZOOM,
            "min_clearance_m": _settings["min_clearance_m"],
        }

    tiles, size = store.stats()
    logger.info(
        f"Terrain data in {cache_dir}"
        + (" (offline: no downloads)" if store.offline else "")
        + (
            f"; extra data folders: {configuration.get('data_dirs')}"
            if configuration.get("data_dirs")
            else ""
        )
        + f"; {tiles} tile(s) available, {size / 1e6:.0f} MB"
    )

    def uavs():
        registry = app.object_registry
        for uav_id in list(registry.ids_by_type(UAV)):
            try:
                yield uav_id, registry.find_by_id(uav_id).status
            except KeyError:
                continue

    def warn(message: str) -> None:
        logger.warning(message)
        app.request_to_send_SYS_MSG_message(message, severity=Severity.WARNING)

    monitor = ClearanceMonitor(uavs, get_ground_heights, warn, _settings, logger)
    _monitor = monitor

    async def watch_clearance():
        """Checks every drone once a second, and fetches terrain tiles for
        drones flying over ground that we have no data for yet."""
        tried: set[tuple[int, int]] = set()
        while True:
            try:
                monitor.step()
                for lat, lon in list(monitor.unknown.values()):
                    cell = (math.floor(lat), math.floor(lon))
                    if cell not in tried and not store.offline:
                        tried.add(cell)
                        await fetch_ground_heights([lat], [lon])
            except Exception:
                logger.exception("Terrain clearance check failed")
            await sleep(1.0)

    async def prefetch_configured_areas():
        """Downloads the tiles of the configured areas, once, at start."""
        for area in configuration.get("prefetch_areas") or []:
            try:
                names = tiles_in_circle(
                    float(area["lat"]),
                    float(area["lon"]),
                    min(float(area.get("radius_km", 20)), 300.0),
                )
            except (KeyError, TypeError, ValueError):
                logger.warning(f"Ignoring invalid terrain prefetch area: {area!r}")
                continue
            summary = await to_thread.run_sync(lambda names=names: store.prefetch(names))
            logger.info(f"Terrain tiles for {area}: {summary}")

    http_server = app.import_api("http_server", HTTPServerExtensionAPI)
    try:
        async with open_nursery() as nursery:
            nursery.start_soon(watch_clearance)
            nursery.start_soon(prefetch_configured_areas)
            with http_server.mounted(blueprint, path="/terrain"):
                await sleep_forever()
    finally:
        _store = None
        _monitor = None
