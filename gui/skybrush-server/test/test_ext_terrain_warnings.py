from logging import getLogger
from types import SimpleNamespace

import numpy as np
import pytest
import trio

from flockwave.server.ext.swarm_bridge.terrain_check import (
    normalise_id,
    terrain_warning,
)
from flockwave.server.ext.terrain.clearance import (
    METRES_PER_DEGREE,
    ClearanceMonitor,
    area_points,
    predict_clearance,
    summarize_clearance,
)

LAT, LON = 12.5, 80.5


def flat(height):
    return lambda lats, lons: np.full(np.shape(np.atleast_1d(lats)), float(height))


def hill_to_the_north(lats, lons):
    """Ground at 100 m, rising to 400 m from 1 km north of (LAT, LON) on."""
    north_m = (np.asarray(lats) - LAT) * METRES_PER_DEGREE
    return np.where(north_m >= 1000, 400.0, 100.0)


# -- predict_clearance ---------------------------------------------------------------


def test_clearance_now_and_ahead_over_flat_ground():
    clearance, when, ground = predict_clearance(LAT, LON, 160.0, 0, 0, 0, 10, flat(100))
    assert clearance == pytest.approx(60.0) and ground == 100.0


def test_a_ridge_ahead_is_found_with_the_time_it_is_reached():
    # flying north at 100 m/s, 1000 m from the ridge: reached after 10 s
    clearance, when, ground = predict_clearance(
        LAT, LON, 350.0, 100, 0, 0, 12, hill_to_the_north
    )
    assert ground == 400.0 and clearance == pytest.approx(-50.0)
    assert when == pytest.approx(10.0)


def test_descending_lowers_the_predicted_altitude():
    # sinking at 5 m/s (down is positive): in 10 s it is 50 m lower
    clearance, when, _ = predict_clearance(LAT, LON, 200.0, 0, 0, 5, 10, flat(100))
    assert clearance == pytest.approx(50.0) and when == pytest.approx(10.0)


def test_unknown_ground_gives_none():
    nothing = lambda lats, lons: np.full(np.shape(np.atleast_1d(lats)), np.nan)
    assert predict_clearance(LAT, LON, 200.0, 0, 0, 0, 10, nothing) is None


def test_summarize_picks_the_highest_ground():
    lats = np.array([12.0, 12.1, 12.2])
    summary = summarize_clearance(lats, lats * 0 + 80, [100, 350, np.nan], 380.0)
    assert summary["min_clearance"] == pytest.approx(30.0)
    assert summary["ground_max"] == 350 and summary["lat"] == 12.1
    assert summary["unknown"] == 1 and summary["samples"] == 3
    assert summarize_clearance(lats, lats, [np.nan] * 3, 100) is None


def test_area_points_cover_the_square():
    lats, lons = area_points(LAT, LON, 600)
    north = (lats - LAT) * METRES_PER_DEGREE
    assert north.min() == pytest.approx(-300, abs=1) and north.max() == pytest.approx(300, abs=1)
    assert len(lats) <= 900 and len(lats) >= 4


# -- the monitor ---------------------------------------------------------------------


def status(ahl=100.0, amsl=200.0, vn=0.0, lat=LAT, lon=LON):
    return SimpleNamespace(
        position=SimpleNamespace(lat=lat, lon=lon, amsl=amsl, ahl=ahl),
        velocity=SimpleNamespace(x=vn, y=0.0, z=0.0),
    )


class Harness:
    def __init__(self, ground, drones):
        self.now = 0.0
        self.sent = []
        self.drones = drones
        self.monitor = ClearanceMonitor(
            lambda: list(self.drones.items()),
            ground,
            self.sent.append,
            {"min_clearance_m": 30.0, "lookahead_s": 10.0},
            getLogger("t"),
            clock=lambda: self.now,
        )


def test_low_drone_is_warned_once_then_repeated_after_a_minute():
    h = Harness(flat(100), {"01": status(amsl=120.0)})  # 20 m above ground
    assert len(h.monitor.step()) == 1
    assert "UAV 01" in h.sent[0] and "20 m" in h.sent[0] and "now" in h.sent[0]
    assert "01" in h.monitor.active

    h.now = 30
    assert h.monitor.step() == []  # not repeated yet
    h.now = 61
    assert len(h.monitor.step()) == 1  # reminder


def test_warning_clears_only_above_limit_plus_hysteresis():
    h = Harness(flat(100), {"01": status(amsl=120.0)})
    h.monitor.step()
    h.drones["01"] = status(amsl=134.0)  # 34 m: above the limit, inside the margin
    h.monitor.step()
    assert "01" in h.monitor.active
    h.drones["01"] = status(amsl=140.0)  # 40 m: clear
    h.monitor.step()
    assert "01" not in h.monitor.active
    h.drones["01"] = status(amsl=120.0)  # low again: a fresh warning
    assert len(h.monitor.step()) == 1


def test_high_drones_and_grounded_drones_are_quiet():
    h = Harness(flat(100), {"01": status(amsl=300.0), "02": status(ahl=0.5, amsl=100.5)})
    assert h.monitor.step() == [] and h.sent == []


def test_a_ridge_ahead_warns_with_the_time():
    h = Harness(hill_to_the_north, {"01": status(amsl=350.0, vn=100.0)})
    assert len(h.monitor.step()) == 1
    assert "in 10 s" in h.sent[0] and "-50 m" in h.sent[0]


def test_unknown_ground_is_reported_for_download_and_not_warned():
    nothing = lambda lats, lons: np.full(np.shape(np.atleast_1d(lats)), np.nan)
    h = Harness(nothing, {"01": status()})
    assert h.monitor.step() == []
    assert h.monitor.unknown == {"01": (LAT, LON)}


def test_drone_that_disappears_is_forgotten():
    h = Harness(flat(100), {"01": status(amsl=120.0)})
    h.monitor.step()
    del h.drones["01"]
    h.monitor.step()
    assert h.monitor.active == {}


# -- command checks in the bridge ---------------------------------------------------


class FakeTerrain:
    """Ground at 100 m everywhere except an area/route that touches 500 m."""

    def __init__(self, ground_max=500.0):
        self.ground_max = ground_max
        self.calls = []

    async def check_route_clearance(self, points, altitude):
        self.calls.append(("route", list(points), altitude))
        return {"min_clearance": altitude - self.ground_max, "ground_max": self.ground_max}

    async def check_area_clearance(self, centers, size, altitude):
        self.calls.append(("area", list(centers), size, altitude))
        return {"min_clearance": altitude - self.ground_max, "ground_max": self.ground_max}


DRONES = {"1": (12.0, 80.0, 10.0), "2": (12.0, 80.1, 20.0), "3": (12.0, 80.2, 15.0)}
HEIGHTS = {"1": 100.0, "2": 200.0, "3": 50.0}  # AMSL: 110, 220, 65


def check(command, uav_ids, kwds, terrain=None, limit=30.0, heights=HEIGHTS, drones=DRONES):
    terrain = terrain or FakeTerrain()
    result = trio.run(
        lambda: terrain_warning(
            command,
            uav_ids,
            kwds,
            drones=drones,
            heights=heights,
            terrain=terrain,
            min_clearance_m=limit,
        )
    )
    return result, terrain


def test_normalise_id():
    assert normalise_id("03") == "3" and normalise_id(3) == "3" and normalise_id("x1") == "x1"


def test_goal_lists_the_drones_that_would_be_too_low_and_uses_home_altitude():
    text, terrain = check(
        "goal", ["01", "02", "03"], {"goal_latlon": [[12.5, 80.5], [12.6, 80.6]]}
    )
    # AMSL altitudes 110, 220 and 65 against ground 500: all far too low
    assert "Goal" in text and "500 m above sea level" in text
    assert "UAV 1 (-390 m)" in text and "UAV 2 (-280 m)" in text and "UAV 3 (-435 m)" in text
    assert "sent anyway" in text
    kinds = [c[0] for c in terrain.calls]
    assert kinds == ["route"] * 3
    # each drone's route starts at the drone itself, then the waypoints
    assert terrain.calls[0][1] == [(12.0, 80.0), (12.5, 80.5), (12.6, 80.6)]
    assert terrain.calls[0][2] == 110.0 and terrain.calls[1][2] == 220.0


def test_enough_clearance_gives_no_warning():
    text, _ = check(
        "goal",
        ["01", "02"],
        {"goal_latlon": [[12.5, 80.5]], "ids": ["1", "2"]},
        terrain=FakeTerrain(ground_max=60.0),
    )
    # 110 - 60 = 50 and 220 - 60 = 160: both above 30
    assert text is None


def test_only_drones_below_the_limit_are_named():
    text, _ = check("search", [], {"center_lat": 12.5, "center_lon": 80.5, "coverage_area": 400},
                    terrain=FakeTerrain(ground_max=100.0))
    # 110-100=10 (low), 220-100=120 (fine), 65-100=-35 (low)
    assert "UAV 1 (+10 m)" in text and "UAV 3 (-35 m)" in text and "UAV 2" not in text


def test_search_and_splits_check_the_area_with_its_size():
    _, terrain = check("search", [], {"center_lat": 12.5, "center_lon": 80.5, "coverage_area": 400})
    assert terrain.calls[0][:3] == ("area", [(12.5, 80.5)], 400.0)
    assert len(terrain.calls) == 3  # every known drone

    _, terrain = check(
        "split", ["01", "02"],
        {"center_lat_lon_array": [[12.5, 80.5], [12.6, 80.6]], "coverage_area": 300,
         "selected_uav_ids": ["1", "2"]},
    )
    assert terrain.calls[0][:3] == ("area", [(12.5, 80.5), (12.6, 80.6)], 300.0)
    assert len(terrain.calls) == 2

    _, terrain = check(
        "specificsplit", [],
        {"center_lat_lon_array": [[12.5, 80.5]], "uav_array": [["1"], ["3"]], "coverage_area": 200},
    )
    assert len(terrain.calls) == 2  # drones 1 and 3 only


def test_unknown_things_are_silent():
    # no terrain data -> no answer; unknown height or drone -> skipped; other commands ignored
    class NoData(FakeTerrain):
        async def check_route_clearance(self, points, altitude):
            return None

    assert check("goal", ["01"], {"goal_latlon": [[12.5, 80.5]]}, terrain=NoData())[0] is None
    assert check("goal", ["01"], {"goal_latlon": [[12.5, 80.5]]}, heights={})[0] is None
    assert check("goal", ["09"], {"goal_latlon": [[12.5, 80.5]]})[0] is None
    assert check("stop", [], {})[0] is None
    assert check("different", [], {"start": 30, "diff": 5})[0] is None
    assert check("goal", ["01"], {"goal_latlon": []})[0] is None
