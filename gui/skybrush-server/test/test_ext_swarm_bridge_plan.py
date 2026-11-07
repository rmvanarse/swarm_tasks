import asyncio

from quart import Quart

from flockwave.server.ext.swarm_bridge.plan_proxy import (
    NO_PLAN,
    PlanProxy,
    make_plan_blueprint,
)

PLAN = {"version": 3, "jobs": [{"id": 1, "kind": "search", "bots": []}]}


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


def run(coro):
    return asyncio.run(coro)


def make_proxy(answers, clock=None, ttl=1.0):
    """A proxy whose fetches come from `answers` (a list: a dict is returned,
    an exception is raised); `calls` counts them."""
    calls = []

    async def fetch(url):
        calls.append(url)
        answer = answers[min(len(calls), len(answers)) - 1]
        if isinstance(answer, Exception):
            raise answer
        return answer

    proxy = PlanProxy("http://main:8765/api/plan", ttl=ttl, fetch=fetch, clock=clock or Clock())
    return proxy, calls


def test_returns_the_plan_of_main_py_and_says_it_is_reachable():
    proxy, calls = make_proxy([PLAN])
    result = run(proxy.get())
    assert result == {**PLAN, "reachable": True}
    assert calls == ["http://main:8765/api/plan"]


def test_fetches_at_most_once_per_ttl_however_many_ask():
    clock = Clock()
    proxy, calls = make_proxy([PLAN, {**PLAN, "version": 4}], clock=clock, ttl=1.0)

    async def three_windows():
        return [await proxy.get() for _ in range(3)]

    assert [r["version"] for r in run(three_windows())] == [3, 3, 3]
    assert len(calls) == 1
    clock.now += 1.5
    assert run(proxy.get())["version"] == 4 and len(calls) == 2


def test_unreachable_main_py_gives_an_empty_plan_with_the_reason():
    proxy, _ = make_proxy([ConnectionError("no route to host")])
    result = run(proxy.get())
    assert result["version"] == NO_PLAN["version"] and result["jobs"] == []
    assert result["reachable"] is False and "no route to host" in result["error"]


def test_comes_back_when_main_py_does():
    clock = Clock()
    proxy, _ = make_proxy([OSError("down"), PLAN], clock=clock)
    assert run(proxy.get())["reachable"] is False
    clock.now += 2
    result = run(proxy.get())
    assert result["reachable"] is True and "error" not in result and result["version"] == 3


def test_a_garbage_answer_is_treated_as_no_plan():
    proxy, _ = make_proxy([{"unexpected": True}])
    result = run(proxy.get())
    assert result["reachable"] is False and result["jobs"] == []


def test_route_serves_the_plan_and_lets_live_read_it_from_another_origin():
    proxy, _ = make_proxy([PLAN])
    app = Quart("t")
    app.register_blueprint(make_plan_blueprint(proxy), url_prefix="/swarm")

    async def go():
        client = app.test_client()
        got = await client.get("/swarm/plan", headers={"Origin": "https://localhost:8080"})
        body = await got.get_json()
        # the pre-flight request a browser sends first
        pre = await client.options(
            "/swarm/plan",
            headers={
                "Origin": "https://localhost:8080",
                "Access-Control-Request-Method": "GET",
            },
        )
        return got, body, pre

    got, body, pre = run(go())
    assert got.status_code == 200 and body["version"] == 3 and body["reachable"] is True
    assert got.headers["Access-Control-Allow-Origin"] == "*"
    assert got.headers["Cache-Control"] == "no-store"
    assert pre.headers["Access-Control-Allow-Origin"] == "*"
