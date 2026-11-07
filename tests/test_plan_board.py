"""The search plan shown in Skybrush Live: the board that holds it and the
records the three mission planners leave for it.

Runs under pytest, or on its own: `python tests/test_plan_board.py`.
"""

import importlib
import math
import os
import sys
import tempfile
import types
from unittest import mock

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

# The planners import these only for features this test does not use.
for _name in ("simplekml", "pymavlink", "pymavlink.mavutil", "yaml"):
    try:
        importlib.import_module(_name)
    except Exception:
        sys.modules[_name] = mock.MagicMock()

from swarm_tasks.plan_board import PlanBoard, square_ring, thin  # noqa: E402

ORIGIN = (12.9288, 80.0464)


def metres(lat1, lon1, lat2, lon2):
    """Ground distance as the mission planners measure it (WGS-84)."""
    from geopy.distance import geodesic

    return geodesic((lat1, lon1), (lat2, lon2)).meters


def bot(n, **extra):
    return {
        "bot": n,
        "sys_id": n + 5,
        "area": 0,
        "strip": {"south": 0, "north": 1, "west": 0, "east": 1},
        "sweeps": [],
        "path": [[12.0, 80.0], [12.1, 80.1]],
        **extra,
    }


def area(ids, lat=12.9, lon=80.0, size=400):
    return {"center": [lat, lon], "size_m": size, "ids": ids}


# -- the board ---------------------------------------------------------------------


def test_square_ring_is_a_closed_square_of_the_right_size():
    ring = square_ring(12.9, 80.0, 400)
    assert len(ring) == 5 and ring[0] == ring[-1]
    lats = [p[0] for p in ring]
    lons = [p[1] for p in ring]
    assert (max(lats) - min(lats)) * 111_320 == pytest_approx(400, 1)
    width = (max(lons) - min(lons)) * 111_320 * math.cos(math.radians(12.9))
    assert width == pytest_approx(400, 1)


def pytest_approx(value, tolerance):
    class _Approx:
        def __eq__(self, other):
            return abs(other - value) <= tolerance

    return _Approx()


def test_thin_keeps_the_ends_and_the_limit():
    points = [[float(i), 0.0] for i in range(1000)]
    out = thin(points, 100)
    assert len(out) <= 101 and out[0] == points[0] and out[-1] == points[-1]
    assert thin(points[:10], 100) == points[:10]


def test_publish_snapshot_and_version():
    board = PlanBoard()
    assert board.snapshot() == {"version": 0, "jobs": []}
    job_id = board.publish("search", "Search", [area([1, 2])], [bot(1), bot(2)])
    snap = board.snapshot()
    assert snap["version"] == 1 and snap["jobs"][0]["id"] == job_id
    job = snap["jobs"][0]
    assert job["kind"] == "search" and [b["bot"] for b in job["bots"]] == [1, 2]
    assert job["areas"][0]["bots"] == [1, 2] and len(job["areas"][0]["ring"]) == 5
    assert {b["status"] for b in job["bots"]} == {"active"}


def test_snapshot_is_a_copy():
    board = PlanBoard()
    board.publish("search", "Search", [area([1])], [bot(1)])
    board.snapshot()["jobs"][0]["bots"][0]["status"] = "hacked"
    assert board.snapshot()["jobs"][0]["bots"][0]["status"] == "active"


def test_a_new_plan_takes_its_bots_out_of_the_old_one():
    board = PlanBoard()
    board.publish("search", "Search", [area([1, 2])], [bot(1), bot(2)])
    board.publish("split", "Split", [area([2, 3], lat=13)], [bot(2), bot(3)])
    jobs = board.snapshot()["jobs"]
    assert [[b["bot"] for b in j["bots"]] for j in jobs] == [[1], [2, 3]]
    assert jobs[0]["areas"][0]["bots"] == [1]


def test_an_area_can_bring_its_own_outline():
    board = PlanBoard()
    ring = [[1, 2], [1, 3], [2, 3], [2, 2], [1, 2]]
    board.publish("search", "Search", [{**area([1]), "ring": ring}], [bot(1)])
    assert board.snapshot()["jobs"][0]["areas"][0]["ring"] == ring


def test_a_job_with_no_bots_left_disappears_with_its_area():
    board = PlanBoard()
    board.publish("search", "Search", [area([1])], [bot(1)])
    board.publish("split", "Split", [area([1])], [bot(1)])
    jobs = board.snapshot()["jobs"]
    assert len(jobs) == 1 and jobs[0]["kind"] == "split"


def test_mark_bots_keeps_the_strip_but_changes_status_and_version():
    board = PlanBoard()
    board.publish("search", "Search", [area([1, 2])], [bot(1), bot(2)])
    before = board.snapshot()["version"]
    board.mark_bots([2], "left")
    snap = board.snapshot()
    statuses = {b["bot"]: b["status"] for b in snap["jobs"][0]["bots"]}
    assert statuses == {1: "active", 2: "left"} and snap["version"] == before + 1
    board.mark_bots([2], "left")  # nothing changed
    assert board.snapshot()["version"] == before + 1
    board.mark_bots([99], "left")  # unknown bot
    assert board.snapshot()["version"] == before + 1


def test_drop_and_clear():
    board = PlanBoard()
    board.publish("search", "Search", [area([1, 2])], [bot(1), bot(2)])
    board.drop_bots([1])
    assert [b["bot"] for b in board.snapshot()["jobs"][0]["bots"]] == [2]
    version = board.snapshot()["version"]
    board.clear()
    assert board.snapshot() == {"version": version + 1, "jobs": []}
    board.clear()  # already empty
    assert board.snapshot()["version"] == version + 1


def test_long_paths_are_thinned():
    board = PlanBoard()
    long_path = [[12.0 + i * 1e-5, 80.0] for i in range(5000)]
    board.publish("search", "Search", [area([1])], [bot(1, path=long_path)])
    path = board.snapshot()["jobs"][0]["bots"][0]["path"]
    assert len(path) <= 401 and path[-1] == long_path[-1]


# -- the planners --------------------------------------------------------------------


def _planner_module(name, tmp):
    module = importlib.import_module(f"swarm_tasks.mission.{name}")
    module.mission_dir = lambda feature, run_id: tmp
    return module


def _check_strips(plan, ids, size_m, centre):
    """The strips of one area stack from south to north, edge to edge, and
    together make the square around `centre`."""
    strips = sorted((plan[i] for i in ids), key=lambda s: s["south"])
    for lower, upper in zip(strips, strips[1:]):
        assert lower["north"] == pytest_approx(upper["south"], 2e-6)
    lon0 = strips[0]["west"]
    height_m = metres(strips[0]["south"], lon0, strips[-1]["north"], lon0)
    assert height_m == pytest_approx(size_m, 1)
    mid_lat = (strips[-1]["north"] + strips[0]["south"]) / 2
    assert mid_lat == pytest_approx(centre[0], 1e-5)
    for s in strips:
        mid = (s["south"] + s["north"]) / 2
        assert metres(mid, s["west"], mid, s["east"]) == pytest_approx(size_m, 1)
        assert s["sweeps"], "every strip has search lines"
        for (la, lo), (lb, mo) in s["sweeps"]:
            # sweeps run east-west (a geodesic drifts about a metre in latitude)
            assert la == pytest_approx(lb, 3e-5)
            assert s["south"] - 1e-5 <= la <= s["north"] + 1e-5
            assert lo == pytest_approx(s["west"], 1e-6)
            assert mo == pytest_approx(s["east"], 5e-6)


def test_auto_split_records_strips_sweeps_and_areas():
    with tempfile.TemporaryDirectory() as tmp:
        mod = _planner_module("groupsplitauto", tmp)
        centres = [[12.93, 80.05], [12.95, 80.08]]
        split = mod.AutoSplitMission(
            origin=ORIGIN,
            center_lat_lons=centres,
            drone_list=[1, 2, 3, 4, 5],
            grid_spacing=50,
            coverage_area=400,
            run_id="t",
        )
        split.GroupSplitting(centres, 5, 50, 400)
        assert sorted(split.plan) == [1, 2, 3, 4, 5]
        assert [a["ids"] for a in split.areas] == [[1, 2, 3], [4, 5]]
        assert [a["center"] for a in split.areas] == centres
        _check_strips(split.plan, [1, 2, 3], 400, centres[0])
        _check_strips(split.plan, [4, 5], 400, centres[1])


def test_auto_split_gives_faster_drones_wider_strips():
    with tempfile.TemporaryDirectory() as tmp:
        mod = _planner_module("groupsplitauto", tmp)
        centre = [[12.93, 80.05]]
        split = mod.AutoSplitMission(
            origin=ORIGIN,
            center_lat_lons=centre,
            drone_list=[1, 2],
            grid_spacing=50,
            coverage_area=400,
            run_id="t",
            drone_speeds={1: 30.0, 2: 10.0},
        )
        split.GroupSplitting(centre, 2, 50, 400)
        height = {i: split.plan[i]["north"] - split.plan[i]["south"] for i in (1, 2)}
        assert height[1] > 2.5 * height[2]


def test_specific_split_records_by_bot_id():
    with tempfile.TemporaryDirectory() as tmp:
        mod = _planner_module("groupsplitspecific", tmp)
        centres = [[12.93, 80.05], [12.95, 80.08]]
        split = mod.SpecificSplitMission(
            origin=ORIGIN,
            center_lat_lons=centres,
            drone_array=[[7, 2], [5]],
            grid_spacing=[50, 50],
            coverage_area=[400, 300],
            run_id="t",
        )
        split.GroupSplitting(centres, [[7, 2], [5]], [50, 50], [400, 300])
        assert sorted(split.plan) == [2, 5, 7]
        assert [a["ids"] for a in split.areas] == [[7, 2], [5]]
        assert [a["size_m"] for a in split.areas] == [400.0, 300.0]
        _check_strips(split.plan, [2, 7], 400, centres[0])
        _check_strips(split.plan, [5], 300, centres[1])


def test_search_records_one_strip_per_drone_number():
    with tempfile.TemporaryDirectory() as tmp:
        mod = _planner_module("bezier_curve", tmp)
        curve = mod.BezierCurveMultiple(ORIGIN, 12.93, 80.05, 3, 50, 500, "t")
        curve.GridFormation()
        assert sorted(curve.plan) == [1, 2, 3]
        strips = sorted(curve.plan.values(), key=lambda s: s["south"])
        # neighbouring strips meet (the planner reserves a gap it does not use,
        # so together they stop short of the full square)
        for lower, upper in zip(strips, strips[1:]):
            assert upper["south"] == pytest_approx(lower["north"], 2e-6)
        total = metres(strips[0]["south"], 80.05, strips[-1]["north"], 80.05)
        assert total <= 500 + 1
        assert all(s["sweeps"] for s in strips)


if __name__ == "__main__":
    failed = 0
    for name, func in sorted(globals().items()):
        if name.startswith("test_") and isinstance(func, types.FunctionType):
            try:
                func()
                print("ok  ", name)
            except Exception as exc:  # noqa: BLE001
                failed += 1
                print("FAIL", name, repr(exc))
    sys.exit(1 if failed else 0)
