"""Swarm operations for the GCS: receives X-SWARM-CMD messages from Skybrush
Live and forwards them to main.py (the swarm code) over UDP.

This extension carries commands only. Drone telemetry reaches Live through the
server's regular mavlink extension (drones stream MAVLink to it directly), and
the same drones stream a second, independent MAVLink feed to main.py.
"""

from __future__ import annotations

from contextlib import ExitStack
from logging import Logger

from trio import sleep_forever

from .driver import SwarmCommandError, SwarmCommandSender
from .schema import schema as schema  # noqa: F401 (picked up by the loader)

description = "Swarm operations (goal, search, split...) forwarded to main.py"
dependencies = ()


async def run(app, configuration, logger: Logger):
    sender = SwarmCommandSender()
    sender.host = str(configuration.get("command_host", "127.0.0.1"))
    sender.port = int(configuration.get("command_port", 12008))
    print("Host: ", str(configuration.get("command_host", "127.0.0.1")))
    sender.id_map = {
        str(k): str(v) for k, v in (configuration.get("id_map") or {}).items()
    }
    sender.default_direction = int(
        configuration.get("default_guided_circle_direction", 1)
    )
    sender.default_radius = int(configuration.get("default_guided_circle_radius", 50))

    def handle_swarm_command(message, client, hub):
        body = message.body
        command = body.get("command")
        try:
            wire = sender.dispatch(command, body.get("uavIds"), body.get("kwds") or {})
        except SwarmCommandError as exc:
            logger.warning(f"swarm_bridge: rejected {command!r}: {exc}")
            return hub.reject(message, reason=str(exc))
        except OSError as exc:
            logger.error(f"swarm_bridge: UDP send failed: {exc!r}")
            return hub.reject(message, reason=f"could not reach swarm code: {exc}")

        logger.info(f"swarm_bridge: sent {wire[:120]!r}")
        return hub.acknowledge(message)

    logger.info(
        f"swarm_bridge: commands go to udp://{sender.host}:{sender.port}"
        + (f", id map {sender.id_map}" if sender.id_map else ", no id map")
    )

    with ExitStack() as stack:
        stack.enter_context(
            app.message_hub.use_message_handlers({"X-SWARM-CMD": handle_swarm_command})
        )
        await sleep_forever()
