"""Hands the search plan from main.py to Skybrush Live.

main.py serves the plan (each drone's strip of the search area, its search lines
and its flight path) at /api/plan on its config panel. Live is a web page and
should not need to reach that machine itself, so the server fetches it and
republishes it at ``GET /swarm/plan``. Fetches are shared and kept for a moment,
so any number of Live windows costs main.py one request a second.
"""

from __future__ import annotations

from time import monotonic
from typing import Any, Awaitable, Callable

import httpx
from quart import Blueprint

from flockwave.server.utils.quart import make_blueprint

__all__ = ("PlanProxy", "make_plan_blueprint")

#: What Live gets while main.py cannot be reached (version -1 never matches a
#: real one, so Live redraws when the connection comes back).
NO_PLAN: dict[str, Any] = {"version": -1, "jobs": []}


class PlanProxy:
    """Latest plan from main.py, fetched at most once per `ttl` seconds."""

    def __init__(
        self,
        url: str,
        *,
        ttl: float = 1.0,
        fetch: Callable[[str], Awaitable[dict]] | None = None,
        clock: Callable[[], float] = monotonic,
    ):
        self.url = url
        self.ttl = ttl
        self._fetch = fetch or self._http_fetch
        self._clock = clock
        self._at: float | None = None
        self._plan: dict[str, Any] = NO_PLAN
        self._error: str | None = None

    @staticmethod
    async def _http_fetch(url: str) -> dict:
        async with httpx.AsyncClient(timeout=1.5) as client:
            response = await client.get(url)
            response.raise_for_status()
            return response.json()

    async def get(self) -> dict[str, Any]:
        """The plan, plus "reachable" (and "error" when it is not)."""
        now = self._clock()
        if self._at is None or now - self._at >= self.ttl:
            self._at = now
            try:
                plan = await self._fetch(self.url)
                if not isinstance(plan, dict) or "jobs" not in plan:
                    raise ValueError("unexpected answer from main.py")
                self._plan, self._error = plan, None
            except Exception as exc:  # noqa: BLE001 - any failure means "no plan"
                self._plan, self._error = NO_PLAN, f"{type(exc).__name__}: {exc}"
        result = {**self._plan, "reachable": self._error is None}
        if self._error:
            result["error"] = self._error
        return result


def make_plan_blueprint(proxy: PlanProxy) -> Blueprint:
    """``GET /plan`` for mounting at ``/swarm``."""
    blueprint = make_blueprint("swarm", __name__)

    @blueprint.after_request
    async def allow_cross_origin(response):
        # Live is served from another origin (its dev server or the Electron
        # shell), so the browser needs this permission to read the answer.
        response.headers["Access-Control-Allow-Origin"] = "*"
        response.headers["Access-Control-Allow-Methods"] = "GET, OPTIONS"
        response.headers["Access-Control-Allow-Headers"] = "Content-Type"
        response.headers["Cache-Control"] = "no-store"
        return response

    @blueprint.route("/plan")
    async def get_plan():
        return await proxy.get()

    return blueprint
