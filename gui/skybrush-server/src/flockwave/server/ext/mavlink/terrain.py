"""Answers the drones' terrain requests using the server's ``terrain`` extension.

ArduPilot keeps the terrain around the vehicle in blocks of 28 x 32 height
samples and asks the ground station for what it is missing: a TERRAIN_REQUEST
carries the block's south-west corner, the sample spacing in metres and a
56-bit mask, one bit for each 4 x 4 piece of the block. The ground station
replies with one TERRAIN_DATA message per set bit.

The layout is the one in ArduPilot's AP_Terrain (TerrainGCS.cpp, TerrainUtil.cpp)
and MAVProxy's terrain module: bit ``b`` is the piece ``b // 8`` steps north and
``b % 8`` steps east of the corner, and ``data[x * 4 + y]`` is the sample ``x``
spacings north and ``y`` spacings east of that piece's corner. Heights are
metres above mean sea level.

Blocks are sent a few per second, like MAVProxy does, so a radio link is not
flooded; a newer request from the same drone replaces an older one that is still
being answered. A block whose heights are not known (no data for that area, and
the server is offline) is skipped and never sent as zeros, so the drone does not
mistake mountains for sea level.
"""

from __future__ import annotations

import math
from logging import Logger
from typing import TYPE_CHECKING, Any

import numpy as np
from trio import CancelScope, Nursery, sleep

from .types import spec

if TYPE_CHECKING:
    from .driver import MAVLinkUAV

__all__ = ("TerrainResponder", "block_points")

#: Metres per degree of latitude, as ArduPilot computes it (its
#: LOCATION_SCALING_FACTOR), so the points line up with the drone's own grid.
METRES_PER_DEGREE = 111318.845

#: Number of 4 x 4 pieces in a block.
PIECES = 56
PIECES_PER_ROW = 8
PIECE_SIZE = 4

#: How many TERRAIN_DATA messages are sent per second to one drone.
BLOCKS_PER_SECOND = 20.0


def block_points(lat: float, lon: float, spacing: float, bit: int):
    """Latitudes and longitudes of the 16 samples of piece `bit` of the block
    whose south-west corner is (lat, lon), in the order TERRAIN_DATA wants."""
    north0 = (bit // PIECES_PER_ROW) * PIECE_SIZE * spacing
    east0 = (bit % PIECES_PER_ROW) * PIECE_SIZE * spacing
    x, y = np.divmod(np.arange(PIECE_SIZE * PIECE_SIZE), PIECE_SIZE)
    north = north0 + x * spacing
    east = east0 + y * spacing
    scale = max(math.cos(math.radians(lat)), 0.01)
    return (
        lat + north / METRES_PER_DEGREE,
        lon + east / (METRES_PER_DEGREE * scale),
    )


class TerrainResponder:
    """Serves TERRAIN_REQUEST messages for all drones of one MAVLink network."""

    def __init__(self, app: Any, driver: Any, nursery: Nursery, log: Logger):
        self._app = app
        self._driver = driver
        self._nursery = nursery
        self._log = log
        self._scopes: dict[Any, CancelScope] = {}
        self._warned_missing_extension = False

    def handle_request(self, uav: "MAVLinkUAV", message) -> None:
        """Starts answering a TERRAIN_REQUEST (called from the message loop)."""
        if not message.mask or message.grid_spacing <= 0:
            return

        api = self._app.import_api("terrain")
        if not api.loaded:
            if not self._warned_missing_extension:
                self._warned_missing_extension = True
                self._log.info(
                    "A drone asked for terrain data but the terrain extension "
                    "is not loaded"
                )
            return

        previous = self._scopes.pop(uav.id, None)
        if previous is not None:
            previous.cancel()
        scope = CancelScope()
        self._scopes[uav.id] = scope
        self._nursery.start_soon(self._serve, uav, message, api, scope)

    async def _serve(self, uav: "MAVLinkUAV", request, api, scope: CancelScope):
        sent = skipped = 0
        try:
            with scope:
                lat, lon = request.lat / 1e7, request.lon / 1e7
                spacing = request.grid_spacing
                for bit in range(PIECES):
                    if not (request.mask >> bit) & 1:
                        continue
                    lats, lons = block_points(lat, lon, spacing, bit)
                    heights = await api.fetch_ground_heights(lats, lons)
                    if not np.all(np.isfinite(heights)):
                        skipped += 1
                        continue

                    data = np.clip(np.rint(heights), -32768, 32767).astype(int).tolist()
                    await self._driver.send_packet(
                        spec.terrain_data(
                            lat=request.lat,
                            lon=request.lon,
                            grid_spacing=spacing,
                            gridbit=bit,
                            data=data,
                        ),
                        uav,
                    )
                    sent += 1
                    await sleep(1.0 / BLOCKS_PER_SECOND)
        except Exception:
            self._log.exception(f"Error while sending terrain data to {uav.id}")
        finally:
            if self._scopes.get(uav.id) is scope:
                del self._scopes[uav.id]

        if skipped:
            self._log.warning(
                f"Terrain data for {uav.id}: {skipped} block(s) skipped, no "
                f"elevation data around {request.lat / 1e7:.4f}, "
                f"{request.lon / 1e7:.4f} (download it with the terrain "
                f"extension while online)"
            )
        elif sent:
            self._log.info(
                f"Sent {sent} terrain block(s) to {uav.id} for "
                f"{request.lat / 1e7:.4f}, {request.lon / 1e7:.4f}"
            )
