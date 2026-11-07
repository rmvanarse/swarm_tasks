"""Map tiles drawn from the elevation data: shaded relief that can be laid over
any base map. The tiles are transparent where the ground is flat, dark on
slopes facing away from the light (north-west) and light on slopes facing it."""

from __future__ import annotations

import math
import struct
import zlib

import numpy as np

from .dem import TerrainStore

__all__ = ("STYLES", "MIN_ZOOM", "MAX_ZOOM", "render_tile", "tile_bounds", "png_rgba")

STYLES = ("hillshade", "relief")
TILE_SIZE = 256
#: Below this zoom a tile would need many source tiles and shows little.
MIN_ZOOM = 8
MAX_ZOOM = 17

_ZENITH = math.radians(45.0)  # sun 45 degrees above the horizon
_AZIMUTH = math.radians((360 - 315 + 90) % 360)  # light from the north-west
_FLAT = math.cos(_ZENITH)  # brightness of flat ground

# Height tint for the "relief" style: metres above sea level -> colour.
_RAMP_M = np.array([0.0, 100.0, 500.0, 1500.0, 3000.0, 5000.0])
_RAMP_RGB = np.array(
    [
        [76, 150, 80],
        [140, 180, 90],
        [220, 200, 120],
        [170, 120, 70],
        [235, 235, 235],
        [255, 255, 255],
    ],
    dtype=float,
)


def vertical_exaggeration(z: int) -> float:
    """How much the heights are stretched before shading. Zoomed in, a pixel is
    a metre or two wide and real slopes are far too gentle to show, so the
    stretch grows with the zoom."""
    return 2.0 + max(0, z - 11)


def contour_interval(z: int) -> float:
    """Height step between contour lines (metres), finer the closer you are."""
    for max_zoom, step in ((9, 200.0), (11, 100.0), (12, 50.0), (13, 20.0), (14, 10.0), (15, 5.0)):
        if z <= max_zoom:
            return step
    return 2.0


def tile_bounds(z: int, x: int, y: int) -> tuple[float, float, float, float]:
    """(north, south, west, east) of a web-mercator tile, in degrees."""
    n = 2.0**z

    def lat_of(ty):
        return math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * ty / n))))

    return lat_of(y), lat_of(y + 1), x / n * 360.0 - 180.0, (x + 1) / n * 360.0 - 180.0


def png_rgba(pixels: np.ndarray) -> bytes:
    """Encodes an (h, w, 4) uint8 array as a PNG."""
    height, width, _ = pixels.shape

    def chunk(kind: bytes, data: bytes) -> bytes:
        body = kind + data
        return (
            struct.pack(">I", len(data))
            + body
            + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF)
        )

    rows = np.zeros((height, 1 + width * 4), dtype=np.uint8)
    rows[:, 1:] = pixels.reshape(height, width * 4)
    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 6, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(rows.tobytes(), 6))
        + chunk(b"IEND", b"")
    )


def _transparent() -> bytes:
    return png_rgba(np.zeros((TILE_SIZE, TILE_SIZE, 4), dtype=np.uint8))


def render_tile(
    store: TerrainStore,
    z: int,
    x: int,
    y: int,
    style: str = "hillshade",
    *,
    download: bool = False,
    size: int = TILE_SIZE,
    contours: bool = False,
) -> bytes:
    """Renders one map tile as a PNG (fully transparent when there is no data
    or the zoom is outside MIN_ZOOM..MAX_ZOOM). With `contours`, thin height
    lines are drawn on top, so even flat ground shows how high it is."""
    if not (MIN_ZOOM <= z <= MAX_ZOOM) or not (0 <= x < 2**z and 0 <= y < 2**z):
        return _transparent()

    north, south, west, east = tile_bounds(z, x, y)

    # Sample one pixel beyond every edge so the slopes at the border are right.
    # Pixel centres are evenly spaced in web-mercator y, not in latitude.
    n = 2.0**z
    px = (np.arange(-1, size + 1) + 0.5) / size
    lons = west + px * (east - west)
    merc_y = (y + px) / n
    lats = np.degrees(np.arctan(np.sinh(math.pi * (1 - 2 * merc_y))))
    lat_grid, lon_grid = np.meshgrid(lats, lons, indexing="ij")
    height = store.elevations(lat_grid, lon_grid, download=download)
    if not np.isfinite(height).any():
        return _transparent()

    # Missing samples (ocean, gaps) count as sea level for shading purposes.
    land = np.nan_to_num(height, nan=0.0)
    land = np.maximum(land, 0.0)

    metres_per_pixel = (
        40075016.686 * math.cos(math.radians((north + south) / 2)) / (size * n)
    )
    stretch = vertical_exaggeration(z)
    dzdx = stretch * (land[1:-1, 2:] - land[1:-1, :-2]) / (2 * metres_per_pixel)
    dzdy = stretch * (land[2:, 1:-1] - land[:-2, 1:-1]) / (2 * metres_per_pixel)  # south - north
    slope = np.arctan(np.hypot(dzdx, dzdy))
    aspect = np.arctan2(dzdy, -dzdx)
    shade = math.cos(_ZENITH) * np.cos(slope) + math.sin(_ZENITH) * np.sin(
        slope
    ) * np.cos(_AZIMUTH - aspect)
    delta = shade - _FLAT

    centre = land[1:-1, 1:-1]
    known = np.isfinite(height[1:-1, 1:-1]) & (centre > 0.0)
    out = np.zeros((size, size, 4), dtype=np.uint8)

    if style == "relief":
        rgb = np.stack(
            [np.interp(centre, _RAMP_M, _RAMP_RGB[:, i]) for i in range(3)], axis=-1
        )
        rgb = np.clip(rgb * (1.0 + delta[..., None] * 1.6), 0, 255)
        out[..., :3] = rgb.astype(np.uint8)
        out[..., 3] = np.where(known, 150, 0).astype(np.uint8)
    else:
        dark = np.clip(-delta * 1.7, 0.0, 0.75)
        light = np.clip(delta * 1.2, 0.0, 0.5)
        out[..., :3] = np.where((delta < 0)[..., None], 0, 255).astype(np.uint8)
        out[..., 3] = np.where(known, np.maximum(dark, light) * 255, 0).astype(np.uint8)

    if contours:
        step = contour_interval(z)
        band = np.floor(land / step)
        # A pixel is on a line when the next pixel (south or east) is in a
        # different height band. The padded border keeps tiles seamless.
        crosses = (band[1:-1, 1:-1] != band[2:, 1:-1]) | (
            band[1:-1, 1:-1] != band[1:-1, 2:]
        )
        crosses &= known
        # Every fifth line is an index contour, drawn darker.
        major = crosses & (np.maximum(band[1:-1, 1:-1], band[2:, 1:-1]) % 5 == 0)
        out[crosses] = (60, 40, 20, 150)
        out[major] = (40, 25, 10, 230)

    return png_rgba(out)
