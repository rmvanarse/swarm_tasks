"""On-disk store for cached map tiles."""

from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass
from hashlib import sha256
from pathlib import Path

__all__ = ("CachedTile", "TileStore")


@dataclass(frozen=True)
class CachedTile:
    """A tile read back from the store."""

    data: bytes
    content_type: str
    fetched_at: float

    def age(self) -> float:
        """Age of the tile in seconds."""
        return max(0.0, time.time() - self.fetched_at)


class TileStore:
    """Stores tiles as a pair of files per tile: ``<hash>.bin`` with the raw
    bytes and ``<hash>.json`` with the content type and the fetch time.

    All methods are blocking; call them from a worker thread.
    """

    def __init__(self, root: Path, max_bytes: int = 0):
        self._root = root
        self._max_bytes = max_bytes
        self._writes_since_prune = 0

    @staticmethod
    def key_for(url: str) -> str:
        return sha256(url.encode("utf-8")).hexdigest()

    def _paths(self, key: str) -> tuple[Path, Path]:
        folder = self._root / key[:2]
        return folder / f"{key}.bin", folder / f"{key}.json"

    def get(self, key: str) -> CachedTile | None:
        bin_path, meta_path = self._paths(key)
        try:
            meta = json.loads(meta_path.read_text(encoding="utf-8"))
            data = bin_path.read_bytes()
            return CachedTile(
                data=data,
                content_type=str(meta.get("content_type") or "application/octet-stream"),
                fetched_at=float(meta.get("fetched_at", 0)),
            )
        except (OSError, ValueError):
            return None

    def put(self, key: str, data: bytes, content_type: str) -> None:
        bin_path, meta_path = self._paths(key)
        bin_path.parent.mkdir(parents=True, exist_ok=True)

        # Write to temp files first so that a crash cannot leave a torn tile
        # behind that would later be served as a valid one.
        tmp_bin = bin_path.with_suffix(".bin.tmp")
        tmp_meta = meta_path.with_suffix(".json.tmp")
        tmp_bin.write_bytes(data)
        tmp_meta.write_text(
            json.dumps({"content_type": content_type, "fetched_at": time.time()}),
            encoding="utf-8",
        )
        os.replace(tmp_bin, bin_path)
        os.replace(tmp_meta, meta_path)

        self._writes_since_prune += 1
        if self._max_bytes > 0 and self._writes_since_prune >= 200:
            self._writes_since_prune = 0
            self.prune()

    def stats(self) -> tuple[int, int]:
        """Returns the number of cached tiles and their total size in bytes."""
        count = 0
        size = 0
        for path in self._root.glob("*/*.bin"):
            try:
                size += path.stat().st_size
                count += 1
            except OSError:
                pass
        return count, size

    def clear(self) -> int:
        """Removes every cached tile; returns the number of tiles removed."""
        removed = 0
        for path in self._root.glob("*/*.*"):
            try:
                path.unlink()
                if path.suffix == ".bin":
                    removed += 1
            except OSError:
                pass
        return removed

    def prune(self) -> None:
        """Deletes the least recently written tiles until the store fits in
        the configured size limit."""
        if self._max_bytes <= 0:
            return

        entries: list[tuple[float, int, Path]] = []
        total = 0
        for path in self._root.glob("*/*.bin"):
            try:
                st = path.stat()
            except OSError:
                continue
            entries.append((st.st_mtime, st.st_size, path))
            total += st.st_size

        if total <= self._max_bytes:
            return

        entries.sort()  # oldest first
        for _, size, path in entries:
            if total <= self._max_bytes:
                break
            try:
                path.unlink()
                path.with_suffix(".json").unlink(missing_ok=True)
                total -= size
            except OSError:
                pass
