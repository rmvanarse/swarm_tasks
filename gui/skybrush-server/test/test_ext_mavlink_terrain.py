import math
from logging import getLogger
from types import SimpleNamespace

import numpy as np
import pytest
import trio

from flockwave.server.ext.mavlink.terrain import (
    METRES_PER_DEGREE,
    TerrainResponder,
    block_points,
)

LAT0, LON0 = 12.5, 80.5
SPACING = 30


def offsets_m(lats, lons, lat0=LAT0, lon0=LON0):
    """(north, east) metres of the points from the corner, as ArduPilot sees them."""
    north = (np.asarray(lats) - lat0) * METRES_PER_DEGREE
    east = (np.asarray(lons) - lon0) * METRES_PER_DEGREE * math.cos(math.radians(lat0))
    return north, east


# -- geometry (layout taken from ArduPilot's TerrainGCS.cpp / TerrainUtil.cpp) ----


def test_piece_zero_starts_at_the_corner_with_x_north_and_y_east():
    north, east = offsets_m(*block_points(LAT0, LON0, SPACING, 0))
    # data[x * 4 + y]: x steps go north, y steps go east
    assert north[0] == pytest.approx(0, abs=1e-6) and east[0] == pytest.approx(0, abs=1e-6)
    assert east[1] == pytest.approx(SPACING, abs=1e-3) and north[1] == pytest.approx(0, abs=1e-6)
    assert north[4] == pytest.approx(SPACING, abs=1e-3) and east[4] == pytest.approx(0, abs=1e-6)
    assert north[15] == pytest.approx(3 * SPACING, abs=1e-3)
    assert east[15] == pytest.approx(3 * SPACING, abs=1e-3)


@pytest.mark.parametrize(
    "bit, pieces_north, pieces_east",
    [(1, 0, 1), (7, 0, 7), (8, 1, 0), (9, 1, 1), (55, 6, 7)],
)
def test_bit_selects_the_piece_row_major_with_eight_per_row(bit, pieces_north, pieces_east):
    north, east = offsets_m(*block_points(LAT0, LON0, SPACING, bit))
    assert north[0] == pytest.approx(pieces_north * 4 * SPACING, abs=1e-3)
    assert east[0] == pytest.approx(pieces_east * 4 * SPACING, abs=1e-3)


def test_the_last_piece_ends_where_the_block_ends():
    # a block is 28 samples north (7 pieces) by 32 east (8 pieces)
    north, east = offsets_m(*block_points(LAT0, LON0, SPACING, 55))
    assert north[15] == pytest.approx(27 * SPACING, abs=1e-3)
    assert east[15] == pytest.approx(31 * SPACING, abs=1e-3)


# -- the responder ------------------------------------------------------------------


class FakeDriver:
    def __init__(self):
        self.sent = []

    async def send_packet(self, packet, uav, **kwargs):
        self.sent.append((uav.id, packet))


def height_at(lats, lons):
    """A synthetic landscape: 1 m of height per 10 m north plus 1 m per 20 m east."""
    north, east = offsets_m(lats, lons)
    return 100 + north / 10 + east / 20


def make_api(loaded=True, nan_for=None):
    calls = []

    async def fetch_ground_heights(lats, lons):
        calls.append(len(lats))
        heights = height_at(lats, lons)
        if nan_for is not None and nan_for(lats, lons):
            heights = np.full_like(heights, np.nan)
        return heights

    return SimpleNamespace(
        loaded=loaded, fetch_ground_heights=fetch_ground_heights, calls=calls
    )


def request(mask, lat=LAT0, lon=LON0, spacing=SPACING):
    return SimpleNamespace(
        lat=int(lat * 1e7), lon=int(lon * 1e7), grid_spacing=spacing, mask=mask
    )


async def run_responder(api, requests, *, wait=0.5):
    driver = FakeDriver()
    app = SimpleNamespace(import_api=lambda name: api)
    uav = SimpleNamespace(id="01")
    async with trio.open_nursery() as nursery:
        responder = TerrainResponder(app, driver, nursery, getLogger("test"))
        for req in requests:
            responder.handle_request(uav, req)
        await trio.sleep(wait)
        nursery.cancel_scope.cancel()
    return driver


def test_sends_one_terrain_data_per_requested_bit_with_the_right_heights():
    mask = (1 << 0) | (1 << 9) | (1 << 55)
    driver = trio.run(run_responder, make_api(), [request(mask)])

    assert [p[1][1]["gridbit"] for p in driver.sent] == [0, 9, 55]
    for _uav, (name, fields) in driver.sent:
        assert name == "TERRAIN_DATA"
        assert fields["lat"] == int(LAT0 * 1e7) and fields["lon"] == int(LON0 * 1e7)
        assert fields["grid_spacing"] == SPACING
        assert len(fields["data"]) == 16 and all(isinstance(v, int) for v in fields["data"])
        expected = np.rint(height_at(*block_points(LAT0, LON0, SPACING, fields["gridbit"])))
        assert fields["data"] == expected.astype(int).tolist()

    # piece 9 is one piece north and one east: 4*30/10 + 4*30/20 = 12 + 6 = 18 m above 100
    assert driver.sent[1][1][1]["data"][0] == 118


def test_unknown_heights_are_skipped_never_sent_as_zero():
    # no data for the piece that starts 4 spacings east of the corner (bit 1)
    def no_data_there(lats, lons):
        north, east = offsets_m(lats, lons)
        return east[0] > 3.5 * SPACING and east[0] < 4.5 * SPACING and north[0] < 1

    driver = trio.run(
        run_responder, make_api(nan_for=no_data_there), [request((1 << 0) | (1 << 1) | (1 << 2))]
    )
    assert [p[1][1]["gridbit"] for p in driver.sent] == [0, 2]


def test_nothing_is_sent_when_the_terrain_extension_is_not_loaded_or_nothing_is_requested():
    driver = trio.run(run_responder, make_api(loaded=False), [request(0b1)])
    assert driver.sent == []
    driver = trio.run(run_responder, make_api(), [request(0)])
    assert driver.sent == []
    driver = trio.run(run_responder, make_api(), [request(0b1, spacing=0)])
    assert driver.sent == []


def test_a_newer_request_from_the_same_drone_replaces_the_old_one():
    first = request(sum(1 << b for b in range(40)))  # a long job
    second = request(1 << 3, lat=LAT0 + 0.01)

    async def scenario():
        driver = FakeDriver()
        api = make_api()
        app = SimpleNamespace(import_api=lambda name: api)
        uav = SimpleNamespace(id="01")
        async with trio.open_nursery() as nursery:
            responder = TerrainResponder(app, driver, nursery, getLogger("test"))
            responder.handle_request(uav, first)
            await trio.sleep(0.12)  # a couple of blocks go out
            before = len(driver.sent)
            responder.handle_request(uav, second)
            await trio.sleep(0.3)
            nursery.cancel_scope.cancel()
        return driver, before

    driver, before = trio.run(scenario)
    assert 1 <= before < 40
    later = [p for p in driver.sent[before:] if p[1][1]["lat"] == int((LAT0 + 0.01) * 1e7)]
    assert [p[1][1]["gridbit"] for p in later] == [3]
    # the first job stopped: far fewer than its 40 bits were sent in total
    assert len(driver.sent) < 40
