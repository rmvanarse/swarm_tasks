"""Builds the exact UDP text commands main.py's command loop expects (see
main.py's `data.startswith(...)` branches and send_command_example.py) and
sends them, one datagram per command tagged with `|id=<token>`. main.py
replies `ACK <id> received`, then `ACK <id> accepted` or `REJECT <id> <reason>`.

Telemetry does not go through here. Drones stream MAVLink straight to the
server's mavlink extension; this module only carries swarm operations.
"""

from __future__ import annotations

import json
import socket
import time
import uuid
from typing import Any, Mapping

# Suffix main.py strips from a command and echoes in its replies.
ID_MARK = "|id="


class SwarmCommandError(ValueError):
    """Raised for an unknown command or missing/invalid arguments."""


class SwarmNoResponse(OSError):
    """main.py did not acknowledge a command in time."""


def _as_wire_id(swarm_id: str) -> Any:
    # main.py parses some ids as ints (split) and others as strings (goal);
    # keep purely numeric ids numeric.
    return int(swarm_id) if swarm_id.isdigit() else swarm_id


class SwarmCommandSender:
    host: str = "127.0.0.1"
    port: int = 12008
    id_map: Mapping[str, str] = {}
    default_direction: int = 1
    default_radius: int = 50

    received_timeout: float = 1.5  # main.py must confirm receipt within this
    result_timeout: float = 20.0  # ...and accept/reject within this

    def send_udp(self, message: str) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.sendto(message.encode("utf-8"), (self.host, self.port))
        finally:
            sock.close()

    def send_and_wait(self, message: str) -> tuple[str, str]:
        """Sends `message` tagged with a fresh command id and waits for
        main.py's replies on the same socket (blocking; call from a worker
        thread). Returns ("accepted", "") or ("rejected", reason), or
        ("received", "") if main.py confirmed receipt but was still busy when
        `result_timeout` ran out. Raises SwarmNoResponse when nothing came
        back within `received_timeout`."""
        cmd_id = uuid.uuid4().hex[:8]
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.sendto(
                f"{message}{ID_MARK}{cmd_id}".encode("utf-8"), (self.host, self.port)
            )
            received = False
            deadline = time.monotonic() + self.received_timeout
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    if received:
                        return "received", ""
                    raise SwarmNoResponse(
                        f"no reply from swarm code at {self.host}:{self.port} "
                        f"(is main.py running?)"
                    )
                sock.settimeout(remaining)
                try:
                    data, _ = sock.recvfrom(4096)
                except socket.timeout:
                    continue  # loop re-checks the deadline
                except ConnectionResetError:
                    # Windows reports an ICMP "port unreachable" this way:
                    # nothing is listening on the swarm code's port.
                    raise SwarmNoResponse(
                        f"swarm code not listening at {self.host}:{self.port} "
                        f"(is main.py running?)"
                    ) from None
                parts = data.decode("utf-8", "replace").split(" ", 3)
                if len(parts) < 3 or parts[1] != cmd_id:
                    continue  # not ours
                if parts[0] == "REJECT":
                    return "rejected", " ".join(parts[2:])
                if parts[2] == "accepted":
                    return "accepted", ""
                if parts[2] == "received" and not received:
                    received = True
                    deadline = time.monotonic() + self.result_timeout
        finally:
            sock.close()

    def swarm_ids(self, ids) -> list[str]:
        """Translates Skybrush UAV ids into main.py's ids: `id_map` wins;
        otherwise zero-padded numeric ids ("01") become plain ("1"), which is
        how the mavlink extension names UAVs versus how main.py does."""
        out = []
        for i in ids or []:
            key = str(i)
            if key in self.id_map:
                out.append(str(self.id_map[key]))
            elif key.isdigit():
                out.append(str(int(key)))
            else:
                out.append(key)
        return out

    def build(self, command: str, uav_ids, kwds: Mapping[str, Any]) -> str:
        """Returns the wire string for `command`. `uav_ids` are Skybrush ids
        (the GUI selection) and are translated through `id_map`."""
        builder = getattr(self, f"_build_{command}", None) if command else None
        if builder is None:
            raise SwarmCommandError(f"unknown swarm command: {command!r}")
        try:
            return builder(list(uav_ids or []), **kwds)
        except TypeError as exc:
            raise SwarmCommandError(f"bad arguments for {command!r}: {exc}") from exc

    def dispatch(self, command: str, uav_ids, kwds: Mapping[str, Any]) -> str:
        """Fire-and-forget variant; returns the wire string."""
        message = self.build(command, uav_ids, kwds)
        self.send_udp(message)
        return message

    # -- one builder per main.py wire command ---------------------------

    def _build_goal(self, uav_ids, goal_latlon, direction=None, radius=None, ids=None):
        # goal_<json [[lat, lon], ...]>_<direction>_<radius>_<json ids>
        targets = self.swarm_ids(ids if ids is not None else uav_ids)
        return "goal_{}_{}_{}_{}".format(
            json.dumps(goal_latlon),
            direction if direction is not None else self.default_direction,
            radius if radius is not None else self.default_radius,
            json.dumps(targets),
        )

    def _build_stop(self, uav_ids):
        return "stop"

    def _build_search(
        self, uav_ids, center_lat, center_lon, grid_space, coverage_area, num_uavs=None
    ):
        # search,<lat>,<lon>,<num_uavs>,<grid_space>,<coverage_area>
        return f"search,{center_lat},{center_lon},{num_uavs or 0},{grid_space},{coverage_area}"

    def _build_split(
        self,
        uav_ids,
        center_lat_lon_array,
        grid_space,
        coverage_area,
        selected_uav_ids=None,
    ):
        ids = self.swarm_ids(selected_uav_ids if selected_uav_ids is not None else uav_ids)
        return "split_{}_{}_{}_{}".format(
            json.dumps(center_lat_lon_array),
            json.dumps([_as_wire_id(i) for i in ids]),
            json.dumps(grid_space),
            json.dumps(coverage_area),
        )

    def _build_specificsplit(
        self, uav_ids, center_lat_lon_array, uav_array, grid_space, coverage_area
    ):
        # uav_array is a list of per-region id lists, e.g. [[1, 2], [3, 4]]
        mapped = [[_as_wire_id(i) for i in self.swarm_ids(group)] for group in uav_array]
        return "specificsplit_{}_{}_{}_{}".format(
            json.dumps(center_lat_lon_array),
            json.dumps(mapped),
            json.dumps(grid_space),
            json.dumps(coverage_area),
        )

    def _build_different(self, uav_ids, start, diff):
        return f"different,{start},{diff}"

    def _build_remove(self, uav_ids):
        return f"remove_{json.dumps(self.swarm_ids(uav_ids))}"

    def _build_add(self, uav_ids):
        return f"add_{json.dumps(self.swarm_ids(uav_ids))}"
