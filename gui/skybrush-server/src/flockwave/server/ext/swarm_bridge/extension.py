"""Swarm operations for the GCS: receives X-SWARM-CMD messages from Skybrush
Live and forwards them to main.py (the swarm code) over UDP. It also relays the
search plan from main.py to Live at GET /swarm/plan (see plan_proxy.py).

This extension carries commands only. Drone telemetry reaches Live through the
server's regular mavlink extension (drones stream MAVLink to it directly), and
the same drones stream a second, independent MAVLink feed to main.py.
"""

from __future__ import annotations

from contextlib import ExitStack
from logging import Logger

import httpx
from trio import open_nursery, sleep_forever, to_thread

from flockwave.server.model.log import Severity
from flockwave.server.model.uav import UAV

from .driver import SwarmCommandError, SwarmCommandSender
from .plan_proxy import PlanProxy, make_plan_blueprint
from .schema import schema as schema  # noqa: F401 (picked up by the loader)
from .terrain_check import normalise_id, terrain_warning

description = "Swarm operations (goal, search, split...) forwarded to main.py"
dependencies = ("http_server",)


async def run(app, configuration, logger: Logger):
    sender = SwarmCommandSender()
    sender.host = str(configuration.get("command_host", "127.0.0.1"))
    sender.port = int(configuration.get("command_port", 12008))
    sender.id_map = {
        str(k): str(v) for k, v in (configuration.get("id_map") or {}).items()
    }
    sender.default_direction = int(
        configuration.get("default_guided_circle_direction", 1)
    )
    sender.default_radius = int(configuration.get("default_guided_circle_radius", 50))
    check_terrain = bool(configuration.get("terrain_warnings", True))
    panel_url = f"http://{sender.host}:{int(configuration.get('telemetry_port', 8765))}"
    telemetry_url = f"{panel_url}/api/telemetry"
    plan_proxy = PlanProxy(f"{panel_url}/api/plan")
    http_server = app.import_api("http_server")
    terrain = app.import_api("terrain")

    async def fetch_heights() -> dict[str, float]:
        """Height above home each drone is set to fly at, from main.py."""
        async with httpx.AsyncClient(timeout=1.5) as client:
            data = (await client.get(telemetry_url)).json()
        return {
            normalise_id(bot["id"]): float(bot["height"])
            for bot in data.get("bots", [])
            if bot.get("height") is not None and "id" in bot
        }

    def known_drones() -> dict[str, tuple[float, float, float]]:
        """id -> (lat, lon, home altitude above sea level) of drones whose
        position and altitudes are known."""
        out = {}
        registry = app.object_registry
        for uav_id in list(registry.ids_by_type(UAV)):
            try:
                pos = registry.find_by_id(uav_id).status.position
            except KeyError:
                continue
            if pos is None or pos.amsl is None or pos.ahl is None:
                continue
            if pos.lat is None or pos.lon is None or (pos.lat == 0 and pos.lon == 0):
                continue
            out[normalise_id(uav_id)] = (pos.lat, pos.lon, pos.amsl - pos.ahl)
        return out

    async def warn_about_terrain(command, uav_ids, kwds):
        """Never lets a problem here disturb the command that was just sent."""
        try:
            if not terrain.loaded:
                return
            limit = float(terrain.get_settings()["min_clearance_m"])
            text = await terrain_warning(
                command,
                uav_ids,
                kwds,
                drones=known_drones(),
                heights=await fetch_heights(),
                terrain=terrain,
                min_clearance_m=limit,
            )
            if text:
                logger.warning(f"swarm_bridge: {text}")
                app.request_to_send_SYS_MSG_message(text, severity=Severity.WARNING)
        except Exception as exc:  # noqa: BLE001
            logger.debug(f"swarm_bridge: terrain check skipped: {exc!r}")

    async def handle_swarm_command(message, client, hub):
        body = message.body
        command = body.get("command")
        try:
            wire = sender.build(command, body.get("uavIds"), body.get("kwds") or {})
            # Blocks until main.py answers, so keep it off the event loop.
            status, reason = await to_thread.run_sync(sender.send_and_wait, wire)
        except SwarmCommandError as exc:
            logger.warning(f"swarm_bridge: rejected {command!r}: {exc}")
            return hub.reject(message, reason=str(exc))
        except OSError as exc:
            logger.error(f"swarm_bridge: {command!r} not delivered: {exc!r}")
            return hub.reject(message, reason=f"could not reach swarm code: {exc}")

        if status == "rejected":
            logger.warning(f"swarm_bridge: main.py rejected {command!r}: {reason}")
            return hub.reject(message, reason=reason)

        logger.info(f"swarm_bridge: {status} {wire[:120]!r}")
        if check_terrain:
            nursery.start_soon(
                warn_about_terrain,
                command,
                list(body.get("uavIds") or []),
                dict(body.get("kwds") or {}),
            )
        return hub.acknowledge(message)

    logger.info(
        f"swarm_bridge: commands go to udp://{sender.host}:{sender.port}"
        + (f", id map {sender.id_map}" if sender.id_map else ", no id map")
    )

    async with open_nursery() as nursery:
        with ExitStack() as stack:
            stack.enter_context(
                app.message_hub.use_message_handlers(
                    {"X-SWARM-CMD": handle_swarm_command}
                )
            )
            stack.enter_context(
                http_server.mounted(make_plan_blueprint(plan_proxy), path="/swarm")
            )
            await sleep_forever()
