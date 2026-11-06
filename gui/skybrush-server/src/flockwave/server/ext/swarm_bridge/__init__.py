"""Skybrush server extension that forwards swarm operations (goal, search,
split...) from Skybrush Live to this project's main.py over UDP, in main.py's
exact wire format. Telemetry is not handled here: drones stream MAVLink to the
server's mavlink extension directly.
"""

from .extension import run
from .schema import schema

__all__ = ("dependencies", "description", "run", "schema")

dependencies = ()
description = "Forwards swarm operations from Live to main.py over UDP"
