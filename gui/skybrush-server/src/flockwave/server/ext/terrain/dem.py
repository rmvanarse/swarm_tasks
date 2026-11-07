"""Elevation data for the terrain extension.

The data is NASA SRTM, in the usual ``.hgt`` layout: one file per 1 degree by
1 degree tile, named after its south-west corner (``N12E080``). Each file is a
square grid of big-endian 16-bit integers, in metres, with row 0 on the
northern edge; the 1 arc-second version is 3601 x 3601 samples (about 30 m),
the 3 arc-second version 1201 x 1201 (about 90 m).

Tiles are downloaded on first use (unless the store is offline), kept on disk
and read through memory maps, so a lookup never loads a whole tile. Heights
are metres above mean sea level. Ocean tiles do not exist at the source; points
there simply have no data.
"""

from __future__ import annotations

import gzip
import math
import threading
import time
from collections import OrderedDict
from logging import Logger
from pathlib import Path
from typing import Callable, Iterable

import httpx
import numpy as np

__all__ = ("DEFAULT_SOURCE_URL", "TerrainStore", "tile_name", "tiles_in_circle")

#: Where tiles are downloaded from. ``{name}`` is e.g. ``N12E080``, ``{ns_lat}``
#: is ``N12`` (the folder).
DEFAULT_SOURCE_URL = (
    "https://elevation-tiles-prod.s3.amazonaws.com/skadi/{ns_lat}/{name}.hgt.gz"
)

VOID = -32768
MAX_OPEN_TILES = 16
RETRY_AFTER_SECONDS = 600.0
DOWNLOAD_TIMEOUT = 60.0


def tile_name(lat: float, lon: float) -> str:
    """Name of the 1-degree tile that contains the given point."""
    lat0, lon0 = math.floor(lat), math.floor(lon)
    return (
        f"{'N' if lat0 >= 0 else 'S'}{abs(lat0):02d}"
        f"{'E' if lon0 >= 0 else 'W'}{abs(lon0):03d}"
    )


def tiles_in_circle(lat: float, lon: float, radius_km: float) -> list[str]:
    """Names of every tile touched by the circle around (lat, lon)."""
    dlat = radius_km / 111.32
    dlon = radius_km / (111.32 * max(math.cos(math.radians(lat)), 0.01))
    names = []
    for la in range(math.floor(lat - dlat), math.floor(lat + dlat) + 1):
        for lo in range(math.floor(lon - dlon), math.floor(lon + dlon) + 1):
            if -90 <= la < 90:
                names.append(tile_name(la + 0.5, ((lo + 180) % 360) - 180 + 0.5))
    return names


class TerrainStore:
    """Disk-backed elevation lookup."""

    def __init__(
        self,
        directory: Path,
        *,
        source_url: str = DEFAULT_SOURCE_URL,
        offline: bool = False,
        logger: Logger | None = None,
        fetch: Callable[[str], bytes | None] | None = None,
        data_dirs: Iterable[Path | str] = (),
    ):
        """
        Args:
            directory: where downloaded tiles are kept
            source_url: URL template for downloading a missing tile
            offline: if true, nothing is ever downloaded
            fetch: replacement for the downloader (for tests); takes a URL and
                returns the gzipped tile, or None if the tile does not exist
            data_dirs: extra folders with ready-made `.hgt` tiles (for example
                tiles downloaded by hand from terrain.ardupilot.org). They are
                only read, never written, and are searched before `directory`.
        """
        self._dir = Path(directory)
        self._dir.mkdir(parents=True, exist_ok=True)
        self._extra_dirs = [Path(d) for d in data_dirs]
        self._source_url = source_url
        self.offline = offline
        self._logger = logger
        self._fetch = fetch or self._http_fetch
        self._lock = threading.RLock()
        self._open: OrderedDict[str, np.ndarray | None] = OrderedDict()
        self._failed_at: dict[str, float] = {}

    # -- files ----------------------------------------------------------

    def path_for(self, name: str) -> Path:
        """Where the tile is read from: the first of the extra data folders
        that has it, otherwise the download folder (even if it is not there)."""
        for folder in self._extra_dirs:
            candidate = folder / f"{name}.hgt"
            if candidate.exists():
                return candidate
        return self._dir / f"{name}.hgt"

    def has_tile(self, name: str) -> bool:
        return self.path_for(name).exists()

    def _sea_marker(self, name: str) -> Path:
        return self._dir / f"{name}.sea"

    def is_sea(self, name: str) -> bool:
        """True if the source told us it has no data for this tile, which for
        this dataset means open ocean. Remembered on disk, so it still holds
        when the server is offline later. Such tiles read as height 0."""
        return self._sea_marker(name).exists()

    def available_tiles(self) -> list[str]:
        names = set()
        for folder in [self._dir, *self._extra_dirs]:
            names.update(p.stem for p in folder.glob("*.hgt"))
        return sorted(names)

    def stats(self) -> tuple[int, int]:
        """Number of tiles on disk (download folder and extra data folders) and
        their total size in bytes."""
        files = [self.path_for(name) for name in self.available_tiles()]
        return len(files), sum(f.stat().st_size for f in files)

    # -- downloading ------------------------------------------------------

    @staticmethod
    def _http_fetch(url: str) -> bytes | None:
        response = httpx.get(url, timeout=DOWNLOAD_TIMEOUT, follow_redirects=True)
        if response.status_code in (403, 404):
            return None  # no such tile (open ocean)
        response.raise_for_status()
        return response.content

    def download(self, name: str) -> str:
        """Fetches one tile if it is not on disk. Blocking; call from a thread.

        Returns "present" (already there), "downloaded", "missing" (the source
        has no such tile, e.g. ocean), "offline" or "failed".
        """
        with self._lock:
            if self.has_tile(name):
                return "present"
            if self.is_sea(name):
                return "missing"
            if self.offline:
                return "offline"
            failed = self._failed_at.get(name)
            if failed is not None and time.monotonic() - failed < RETRY_AFTER_SECONDS:
                return "failed"

            url = self._source_url.format(name=name, ns_lat=name[:3])
            try:
                data = self._fetch(url)
            except (httpx.HTTPError, OSError) as exc:
                self._failed_at[name] = time.monotonic()
                if self._logger:
                    self._logger.warning(f"Terrain tile {name} download failed: {exc!r}")
                return "failed"

            if data is None:
                # The source has no such tile: open ocean. Remember that.
                self._sea_marker(name).touch()
                return "missing"

            try:
                raw = gzip.decompress(data)
            except OSError:
                raw = data  # not compressed
            side = math.isqrt(len(raw) // 2)
            if side * side * 2 != len(raw) or side < 2:
                self._failed_at[name] = time.monotonic()
                if self._logger:
                    self._logger.warning(f"Terrain tile {name} has an unexpected size")
                return "failed"

            tmp = self._dir / f"{name}.hgt.part"
            tmp.write_bytes(raw)
            tmp.replace(self._dir / f"{name}.hgt")
            self._open.pop(name, None)
            if self._logger:
                self._logger.info(f"Downloaded terrain tile {name} ({side}x{side})")
            return "downloaded"

    def prefetch(self, names: Iterable[str]) -> dict[str, list[str]]:
        """Downloads every missing tile in `names`. Blocking."""
        summary: dict[str, list[str]] = {}
        for name in dict.fromkeys(names):
            summary.setdefault(self.download(name), []).append(name)
        return summary

    # -- lookups ----------------------------------------------------------

    def _tile(self, name: str) -> np.ndarray | None:
        with self._lock:
            if name in self._open:
                self._open.move_to_end(name)
                return self._open[name]
            path = self.path_for(name)
            if not path.exists():
                return None
            side = math.isqrt(path.stat().st_size // 2)
            array = np.memmap(path, dtype=">i2", mode="r", shape=(side, side))
            self._open[name] = array
            while len(self._open) > MAX_OPEN_TILES:
                self._open.popitem(last=False)
            return array

    def elevations(self, lats, lons, *, download: bool = False) -> np.ndarray:
        """Heights above sea level (metres) at the given points, interpolated
        between grid samples; NaN where there is no data. With `download`, a
        missing tile is fetched first (blocking)."""
        lats = np.atleast_1d(np.asarray(lats, dtype=float))
        lons = np.atleast_1d(np.asarray(lons, dtype=float))
        lats, lons = np.broadcast_arrays(lats, lons)
        out = np.full(lats.shape, np.nan)
        valid = np.isfinite(lats) & np.isfinite(lons) & (np.abs(lats) <= 90)
        lat0 = np.floor(lats)
        lon0 = np.floor(lons)

        pairs = {(la, lo) for la, lo in zip(lat0[valid], lon0[valid])}
        for la, lo in pairs:
            name = tile_name(la + 0.5, lo + 0.5)
            if download and not self.has_tile(name):
                self.download(name)
            tile = self._tile(name)
            sel = valid & (lat0 == la) & (lon0 == lo)
            if tile is None:
                if self.is_sea(name):
                    out[sel] = 0.0  # open ocean
                continue
            n = tile.shape[0]
            row = (la + 1 - lats[sel]) * (n - 1)
            col = (lons[sel] - lo) * (n - 1)
            r0 = np.clip(np.floor(row).astype(int), 0, n - 2)
            c0 = np.clip(np.floor(col).astype(int), 0, n - 2)
            fr = np.clip(row - r0, 0.0, 1.0)
            fc = np.clip(col - c0, 0.0, 1.0)
            a = tile[r0, c0].astype(float)
            b = tile[r0, c0 + 1].astype(float)
            c = tile[r0 + 1, c0].astype(float)
            d = tile[r0 + 1, c0 + 1].astype(float)
            bad = (a == VOID) | (b == VOID) | (c == VOID) | (d == VOID)
            value = (a * (1 - fc) + b * fc) * (1 - fr) + (c * (1 - fc) + d * fc) * fr
            value[bad] = np.nan
            out[sel] = value
        return out

    def elevation(self, lat: float, lon: float, *, download: bool = False):
        """Height above sea level at one point, or None if unknown."""
        value = float(self.elevations([lat], [lon], download=download)[0])
        return None if math.isnan(value) else value
