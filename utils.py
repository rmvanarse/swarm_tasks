from math import cos, sin, radians, degrees
from yaml import safe_load, safe_dump  # type: ignore
from os import path,makedirs
from pymavlink import mavutil

def compass_to_math_rad(heading_deg):
    """
    Converts a compass heading (degrees, 0=N, clockwise)
    to a standard math angle (radians, 0=+x axis, counter-clockwise).
    """
    math_deg = (90 - heading_deg) % 360
    return radians(math_deg)


def math_rad_to_compass(theta_rad):
    """
    Inverse of compass_to_math_rad(): converts a standard math angle
    (radians, 0=+x axis, counter-clockwise) back to a compass heading
    (degrees, 0=N, clockwise).
    """
    heading_deg = (90 - degrees(theta_rad)) % 360
    return heading_deg


def opposite_deg(heading):
    return (heading + 180) % 360


def lookahead_point(x, y, theta, distance):
    lx = x + distance * cos(theta)
    ly = y + distance * sin(theta)
    return (lx, ly)


def compute_lookahead_target(x, y, theta, rx, ry, distance, min_lead=None):
    """
    tx,ty = the point handed to the real vehicle: a fixed point on the
    bot's heading line, `distance` ahead of the bot, that the vehicle
    closes on -- never a point that runs away from it, and never one
    that falls behind it.

    x,y,theta -- bot pose (sim frame)
    rx,ry     -- vehicle's live position (sim frame, same units as x,y)
    distance  -- lookahead distance ahead of the BOT, sim units
    min_lead  -- once the vehicle has passed the bot-anchored target,
                 how far ahead of the vehicle to keep it instead.
                 Defaults to half `distance`.

    `along` is the vehicle's signed progress along the bot's heading:
    >0 means it has already passed the bot going that way. Anchoring
    the target to that progress (target_along = along + distance) is
    what misbehaves once the vehicle gets ahead -- the target then
    advances by exactly as much as the vehicle does, so the vehicle
    chases a carrot it can never reach and keeps building lead on the
    bot. Behind, the same formula is self-stabilising (the gap grows,
    giving a far target and a gentle straight-line pursuit), which is
    why only the "ahead" case shows the problem.

    So `along` is used here purely as a floor, not as the anchor: the
    target normally sits at a FIXED `distance` ahead of the bot, so a
    vehicle that is ahead actually closes on it and bleeds off its
    lead. Only when the vehicle has genuinely overflown that point
    does the target get pushed out, and then just by min_lead -- far
    enough to avoid commanding a reversal a fixed-wing can't fly.
    """
    if min_lead is None:
        min_lead = distance * 0.5

    # Signed distance of vehicle along the bot's current heading
    along = (rx - x) * cos(theta) + (ry - y) * sin(theta)

    target_along = max(distance, along + min_lead)
    return lookahead_point(x, y, theta, target_along)


def _create_placeholder_rectangles_yaml(filepath):
    """First run on a machine that has never had swarm_env/rectangles.yaml
    set up: write a placeholder instead of crashing the whole process on
    a missing-file error before anything else even starts.

    origin is deliberately (0.0, 0.0) -- not this project's own test site
    -- so a forgotten placeholder is obvious on a map rather than a
    plausible-looking wrong location. size/obstacles are left empty; this
    file still needs real survey data before any actual mission planning
    makes sense.
    """
    makedirs(path.dirname(filepath), exist_ok=True)
    placeholder = {
        "name": "Rectangles",
        "size": {"x": 0, "y": 0},
        "obstacles": [],
        "origin": "(0.0, 0.0)",
    }
    with open(filepath, "w") as f:
        safe_dump(placeholder, f, sort_keys=False)
    print(
        f"WARNING: {filepath} did not exist -- created a placeholder "
        "(origin 0.0, 0.0, no obstacles). Edit it with your real survey "
        "site's origin/obstacles before flying a real mission."
    )


def read_origin(
    filepath=path.join(
        path.expanduser("~"), "Documents", "swarm_env", "rectangles.yaml"
    )
):
    """Same approach copter_swarm.py uses: read origin straight from the
    persisted rectangles.yaml on disk at startup, so it's already valid by
    the time the arm+altitude wait loop calls fetch_location() -- no
    hardcoded per-site preset, no polling required."""
    print("Reading YAML from:", filepath)
    if not path.exists(filepath):
        _create_placeholder_rectangles_yaml(filepath)
    with open(filepath) as f:
        data = safe_load(f)
    origin = data.get("origin")
    if isinstance(origin, str):
        origin = origin.strip("()")
        lat, lon = origin.split(",")
        origin = [float(lat), float(lon)]
    return origin


def send_reposition(
    vehicle, lat, lon, alt_rel, loiter_radius=0.0, clockwise=True, groundspeed=-1
):
    """
    Sends ONE MAV_CMD_DO_REPOSITION instead of dronekit's simple_goto().
    Unlike simple_goto (a bare position target with no say over what the
    vehicle does once it arrives), this single command also carries the
    loiter radius/direction to hold there -- ArduPilot flies the intercept
    AND the arrival behaviour from this one command.

    loiter_radius=0.0 (default) tells the firmware "arrive, don't loiter" --
    this is what actually stops the vehicle auto-circling on arrival with
    its own WP_LOITER_RAD, instead of relying on always keeping the
    commanded point artificially far away (see vehicle_loiter_radius_min_m
    in the search loop). Pass a radius (metres) if you DO want it to orbit
    once it gets there, e.g. for the final point of a route.

    alt_rel: metres above home (MAV_FRAME_GLOBAL_RELATIVE_ALT).
    clockwise: loiter direction, only meaningful when loiter_radius>0.
    groundspeed: commanded speed in m/s, -1 = vehicle's current/default.
    """
    master = vehicle._master
    radius = float(loiter_radius) if loiter_radius else 0.0
    yaw = (0.0 if clockwise else 1.0) if loiter_radius else float("nan")
    master.mav.command_int_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_DO_REPOSITION,
        0,
        0,
        groundspeed,
        1,
        radius,
        yaw,
        int(round(lat * 1e7)),
        int(round(lon * 1e7)),
        alt_rel,
    )


def current_process(swarm, stop=False, call_back_action=None):
    """Report the currently running swarm function, and optionally stop it.

    call_back_action is what Thread.stop() invokes to signal the worker
    to wind down -- it defaults to swarm.stopAction, because a stop
    WITHOUT it would set the thread's stop event while the worker sat
    in its own inner `while True`, so the join() below would block
    forever. Pass a different callback only to signal something else.

    Teardown (closing the GUI, clearing per-run state) runs here after
    the join, on the caller's thread -- matplotlib is not thread-safe,
    and the worker must be gone before its leftovers are cleared.
    """
    running = swarm.running_funcs()
    if not running:
        print("Current Process: none")
        return []
    print(f"Current Process: {[name for name, _ in running]}")
    if not stop:
        return []
    # Stop EVERY running function, not just the first -- a stop means
    # stop, and leaving a second worker alive would keep commanding the
    # vehicles after teardown had cleared the state it relies on.
    return swarm.stopAll(call_back_action)


def mission_dir(feature,run_id):
    """Create and return a fresh ~/Documents/swarm_env/missions/<feature>/<run_id>/
    folder for one grid-generation run. Call once per planner-object
    construction and reuse the returned path -- calling again later would
    mint a new run_id and point at an empty folder.

    run_id is timestamp-prefixed for human sorting/browsing, with a short
    random suffix so two calls within the same second (e.g. rapid repeated
    submissions) never collide on the same folder and silently clobber each
    other's output."""

    mission_path = path.join(
        path.expanduser("~"),
        "Documents",
        "swarm_env",
        "missions",
        feature,
        run_id,
    )

    makedirs(mission_path, exist_ok=True)
    return mission_path

def generate_heights(start_height, num_drones, difference):
    return [start_height + i * difference for i in range(num_drones)]
