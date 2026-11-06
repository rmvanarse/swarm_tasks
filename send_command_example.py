"""
Example UDP client for sending mission commands to the swarm backend
(main.py).

main.py listens for commands on UDP port 12008
(swarm_tasks/network/UDP.py's UDPReceiver), bound to the machine's own
LAN IP -- the one it prints at startup ("Ip address: ..."), NOT
127.0.0.1/localhost. Each datagram is decoded as a plain UTF-8 string
and matched against the if/elif chain at the bottom of main.py, so the
wire format below has to match that exactly: same delimiter, same field
order, same JSON-encoded sub-fields. It's fire-and-forget -- main.py's
loop never sends a reply back over this socket.

Usage:
    python send_command_example.py                 # runs the demo below

    from send_command_example import SwarmClient
    client = SwarmClient("192.168.1.42")            # IP main.py printed
    client.search(center_lat=12.93, center_lon=80.02, num_uavs=10,
                  grid_space=50, coverage_area=2000)
    client.close()
"""

import json
import socket
import time

DEFAULT_PORT = 12008


class SwarmClient:
    """Thin UDP sender -- one datagram per command."""

    def __init__(self, host, port=DEFAULT_PORT):
        self.addr = (host, port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def _send(self, message: str):
        print(f"-> {self.addr}: {message}")
        self._sock.sendto(message.encode("utf-8"), self.addr)

    def close(self):
        self._sock.close()

    # -- One method per main.py command branch ---------------------------

    def stop(self):
        """Stops every currently running function (search/goal/split)."""
        self._send("stop")

    def search(self, center_lat, center_lon, num_uavs, grid_space, coverage_area):
        """Area-coverage search with every (non-removed) connected drone.

        Wire format: "search,<center_lat>,<center_lon>,<num_uavs>,
        <grid_space>,<coverage_area>" -- main.py does
        data.split(",", 6) and unpacks the first 6 fields positionally.
        """
        self._send(
            f"search,{center_lat},{center_lon},{num_uavs},{grid_space},{coverage_area}"
        )

    def goal(self, goal_latlon, direction, radius, ids):
        """Sends a set of bots to fly a sequence of lat/lon goals.

        goal_latlon: list of [lat, lon] pairs, flown in order.
        direction/radius: guided-circle direction + radius (metres) --
            whatever your build of swarm.goal()/line_waypoint_guidance
            expects for the final-point loiter.
        ids: list of bot ids (1-based) this command applies to -- a new
            goal for bot 2 does not disturb whatever bot 1 is doing.

        Wire format: "goal_<goal_latlon json>_<direction>_<radius>_<ids json>"
        -- main.py does data.split("_") and unpacks by position.
        """
        self._send(
            f"goal_{json.dumps(goal_latlon)}_{direction}_{radius}_{json.dumps(ids)}"
        )

    def remove(self, bot_ids):
        """Pulls one or more bots out of service (self-heal): their
        remaining goals are handed to whichever bot finishes first.
        bot_ids: a single int or a list of ints."""
        ids = bot_ids if isinstance(bot_ids, list) else [bot_ids]
        self._send(f"remove_{json.dumps(ids)}")

    def add(self, bot_ids):
        """Puts previously removed bot(s) back into service."""
        ids = bot_ids if isinstance(bot_ids, list) else [bot_ids]
        self._send(f"add_{json.dumps(ids)}")

    def split(self, center_lat_lon_array, selected_uav_ids, grid_space, coverage_area):
        """Auto-split mission: divides each area in center_lat_lon_array
        evenly among the bots in selected_uav_ids (by count, or by
        cruise speed if the backend's hybrid-swarm weighting is in use).

        Wire format: "split_<center_lat_lon_array json>_<selected_uav_ids json>
        _<grid_space json>_<coverage_area json>".
        """
        self._send(
            f"split_{json.dumps(center_lat_lon_array)}_{json.dumps(selected_uav_ids)}"
            f"_{json.dumps(grid_space)}_{json.dumps(coverage_area)}"
        )

    def specific_split(
        self, center_lat_lon_array, uav_array, grid_space, coverage_area
    ):
        """Specific-split mission: uav_array is a list of per-area id
        lists (e.g. [[1,2], [3,4]]), one sub-list per entry in
        center_lat_lon_array -- lets you hand-pick which bots cover
        which area instead of an automatic round-robin split.

        Wire format: "specificsplit_<center_lat_lon_array json>_<uav_array json>
        _<grid_space json>_<coverage_area json>".
        """
        self._send(
            f"specificsplit_{json.dumps(center_lat_lon_array)}_{json.dumps(uav_array)}"
            f"_{json.dumps(grid_space)}_{json.dumps(coverage_area)}"
        )

    def different_heights(self, start, diff):
        """Staggers each bot's altitude: heights[i] = start + i*diff
        (metres), so bots flying near each other don't share an altitude."""
        self._send(f"different,{start},{diff}")


if __name__ == "__main__":
    # Replace with the IP main.py printed at startup ("Ip address: ...").
    # NOT 127.0.0.1/localhost -- main.py binds its UDP listener to the
    # machine's real LAN IP, not the loopback interface.
    SWARM_HOST = "192.168.31.15"

    client = SwarmClient(SWARM_HOST)
    try:
        # Demo: kick off a 10-drone area-coverage search, then tell bot 3
        # to hold position (self-heal) a moment later.
        # client.stop()
        # time.sleep(2)
        client.search(
            center_lat=12.9595609,
            center_lon=80.0387478,
            num_uavs=10,
            grid_space=100,
            coverage_area=2000,
        )
        # time.sleep(240)
        # client.goal(goal_latlon=[[12.9483525,80.0587463],[12.9595609,80.0387478],[12.9707692,80.0187492]],direction=1,radius=100,ids=["01","02","08","09"])
        # client.add(3)
        # client.stop()
    finally:
        client.close()
