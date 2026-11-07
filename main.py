import json
from swarm import Swarm
from variable import Variables
from swarm_tasks.network.UDP import UDPReceiver, CallBack
from vehicle import Vehicles
from utils import current_process, generate_heights
from swarm_tasks.config_server import ConfigServer
from swarm_tasks.network.network import NetworkIP
import os
import traceback
from time import sleep


def build_runtime(ip=None, num_bots=15):
    """Construct the swarm runtime: vehicle connections, live-tunable
    config, and the Swarm guidance/kinematics object.

    The one piece of initialization shared by this module's own CLI/UDP
    loop below and the skybrush-server swarm_runtime extension, which
    imports and calls this directly in-process instead of duplicating it
    or talking to a second main.py process over the network.
    """
    ip = ip or NetworkIP.get_ip()
    print(f"Ip address: {ip}")
    vehicles = Vehicles([f"udpin:{ip}:{14551 + i}" for i in range(num_bots)])
    vehicle_loiter_radius_min_m = vehicles.Loiter_param()
    variables = Variables(vehicle_loiter_radius_min_m=vehicle_loiter_radius_min_m)
    print(variables.vehicle_loiter_radius_min_m)
    variables.heights = [100 + i * 10 for i in range(len(vehicles.drones))]
    swarm = Swarm(num_bots=len(vehicles.drones), Variables=variables, vehicle=vehicles)
    return ip, vehicles, variables, swarm


def to_bot_ids(vehicles, ids):
    """Translate drone MAVLink system ids, as sent by the GUI, into the
    swarm's internal bot ids (1-based position in vehicles.drones).

    Swarm addresses bots by position; the outside world (GUI, telemetry)
    addresses drones by system id. Accepts nested lists (specificsplit).
    """
    if isinstance(ids, (list, tuple)):
        return [to_bot_ids(vehicles, i) for i in ids]
    return vehicles.bot_id_for_sysid(ids)


if __name__ == "__main__":
    try:
        callBack = CallBack()
        # SWARM_IP / SWARM_NUM_BOTS override auto-detection, e.g. to bind the
        # vEthernet (WSL) address when SITL runs in WSL on this machine.
        ip, vehicles, variables, swarm = build_runtime(
            ip=os.environ.get("SWARM_IP") or None,
            num_bots=int(os.environ.get("SWARM_NUM_BOTS", "10")),
        )
        receiver = UDPReceiver((ip, 12008), callback=callBack.call_back)
        receiver.start()
        callBack.attach(receiver)
        # Live-editable config panel + read-only /api/telemetry for the GUI
        # bridge. Same Variables object Swarm holds, so an edit lands on the
        # workers' next tick -- no restart. Started only after swarm exists,
        # since /api/telemetry reads it.
        ConfigServer(variables=variables, host=ip, swarm=swarm).start()
        while True:
            # Matplotlib/Qt calls must happen on the main thread; searchSub()
            # (background thread) only publishes state for this to draw.
            swarm.gui_tick()

            data = callBack.get_current_data()

            if data is None:
                # With the GUI off gui_tick() returns at once, so without a
                # pause this loop spins flat out and hogs the GIL, delaying
                # every guidance worker and MAVLink thread (ticks measured
                # ~25% slower on average, up to 4x on single ticks). The
                # plot's own redraw used to pace this loop implicitly.
                sleep(0.01)
                continue

            try:
                if data.startswith("stop"):
                    current_process(swarm, stop=True, call_back_action=swarm.stopAction)
                    callBack.set_data()
                    continue

                elif data.startswith("search"):
                    current_process(swarm, stop=True, call_back_action=swarm.stopAction)
                    print("decoded_index", data)
                    msg_parts = data.split(",", 6)
                    selected_uav_raw = msg_parts[6] if len(msg_parts) > 6 else None
                    _, center_lat, center_lon, num_uavs, grid_space, coverage_area = (
                        msg_parts[:6]
                    )
                    swarm.search(
                        center_lat=center_lat,
                        center_lon=center_lon,
                        effective_num_uavs=len(vehicles.drones),
                        grid_space=grid_space,
                        coverage_area=coverage_area,
                    )
                    callBack.set_data()

                elif data.startswith("goal"):
                    print(f"Goal message received: {data}")
                    # No stop here: goal() stops only the jobs owning the bots
                    # in this message, so a goal for bot 2 leaves bot 1 flying.
                    msg_parts = data.split("_")
                    swarm.goal(
                        guided_circle_direction=msg_parts[2],
                        guided_circle_radius=msg_parts[3],
                        goal_latlon=json.loads(msg_parts[1]),
                        ids=to_bot_ids(vehicles, json.loads(msg_parts[4])),
                    )
                    callBack.set_data()

                elif data.startswith("remove") or data.startswith("add"):
                    # Self-heal control, e.g.  remove_["02"]   or   add_2
                    # A removed bot is frozen and its unflown points go to
                    # whichever bot finishes its own work first.
                    action, _, raw = data.partition("_")
                    try:
                        bot_ids = json.loads(raw)
                    except ValueError:
                        bot_ids = [p for p in raw.replace(",", " ").split() if p]
                    if not isinstance(bot_ids, list):
                        bot_ids = [bot_ids]
                    for bot_id in to_bot_ids(vehicles, bot_ids):
                        if action == "remove":
                            swarm.remove_bot(bot_id)
                        else:
                            swarm.add_bot(bot_id)
                    callBack.set_data()
                    continue

                elif data.startswith("split"):
                    # No global stop: like goal, a split replaces only the
                    # jobs that own its bots, so splits can run side by side.
                    msg_parts = data.split("_")
                    print("msg_parts", msg_parts, len(msg_parts))
                    # Live sends the ids in the order they were selected, but the
                    # split gives the first N bots to point 1, the next N to
                    # point 2 ... so use plain bot order: 1-5 then 6-10.
                    selected_uav_ids = sorted(
                        set(to_bot_ids(vehicles, json.loads(msg_parts[2])))
                    )
                    grid_space = json.loads(msg_parts[3])
                    coverage_area = json.loads(msg_parts[4])
                    center_lat_lon_array = json.loads(
                        msg_parts[1]
                    )  # All other coordinates
                    swarm.AutosplitMission(
                        selected_uav_ids,
                        grid_space,
                        coverage_area,
                        center_lat_lon_array,
                    )
                    callBack.set_data()

                elif data.startswith("specificsplit"):
                    # No global stop: only the jobs owning these bots are
                    # replaced (see swarm._start_route_job).
                    msg_parts = data.split("_")
                    print("msg_parts", msg_parts)
                    center_lat_lon_array = json.loads(
                        msg_parts[1]
                    )  # All other coordinates
                    uav_array = to_bot_ids(vehicles, json.loads(msg_parts[2]))
                    grid_space = json.loads(msg_parts[3])
                    coverage_area = json.loads(msg_parts[4])
                    swarm.SpecificSplit(
                        center_lat_lon_array, uav_array, grid_space, coverage_area
                    )
                    callBack.set_data()

                elif data.startswith("different"):
                    msg_parts = data.split(",")
                    start = int(msg_parts[1])
                    diff = int(msg_parts[2])
                    variables.heights = generate_heights(
                        start_height=start,
                        num_drones=len(vehicles.drones),
                        difference=diff,
                    )
                    print(variables.heights)
                    # Climb/descend the drones that are already flying; the
                    # new heights are otherwise only used by goal/search.
                    print("heights applied to bots:", swarm.apply_heights())
                    callBack.set_data()

                else:
                    print(f"in else data: {data}")
                    callBack.set_data(error=f"unknown command: {data[:40]!r}")
            except Exception as exc:
                # A bad command must not take the whole swarm process down:
                # report it and drop the command so the loop keeps serving.
                # The sender gets the reason as a REJECT reply.
                traceback.print_exc()
                callBack.set_data(error=exc)
    except Exception:
        traceback.print_exc()
