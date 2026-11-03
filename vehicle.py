from concurrent.futures import ThreadPoolExecutor, as_completed

# dronekit (2.9.2, last released 2019) still does `collections.MutableMapping`,
# which was removed from `collections` in Python 3.10 (moved to
# `collections.abc` back in 3.3). Restore the old aliases before importing it
# so `from dronekit import ...` below doesn't crash with AttributeError on
# modern Python.
import collections
import collections.abc

for _name in ("MutableMapping", "Mapping", "Sequence", "Iterable"):
    if not hasattr(collections, _name):
        setattr(collections, _name, getattr(collections.abc, _name))

from dronekit import connect, APIException
from pymavlink import mavutil
from utils import compass_to_math_rad

# Hybrid swarm support: a connected drone's real airframe (quadcopter,
# fixed-wing plane, VTOL, rover, ...) is reported in every MAVLink
# HEARTBEAT as a MAV_TYPE_* value, which dronekit already captures into
# vehicle._vehicle_type as soon as connect() gets its first heartbeat --
# no extra MAVLink traffic needed. We collapse that down to the
# simulation's two kinematic models ('fixedwing': bank-limited turns,
# can't hover/pivot; 'ground': can pivot/hover in place -- covers
# multirotors, helicopters and rovers) so each bot in a mixed swarm is
# simulated with the kinematics matching its own real vehicle, instead of
# one vehicle_type hardcoded for every bot.
_FIXEDWING_MAV_TYPES = {
    mavutil.mavlink.MAV_TYPE_FIXED_WING,
    mavutil.mavlink.MAV_TYPE_VTOL_DUOROTOR,
    mavutil.mavlink.MAV_TYPE_VTOL_QUADROTOR,
    mavutil.mavlink.MAV_TYPE_VTOL_TILTROTOR,
    mavutil.mavlink.MAV_TYPE_VTOL_RESERVED2,
    mavutil.mavlink.MAV_TYPE_VTOL_RESERVED3,
    mavutil.mavlink.MAV_TYPE_VTOL_RESERVED4,
    mavutil.mavlink.MAV_TYPE_VTOL_RESERVED5,
}


def mav_type_to_vehicle_type(mav_type):
    """MAV_TYPE_* (from a drone's HEARTBEAT) -> this project's sim
    vehicle_type ('fixedwing' or 'ground'). Unrecognised/unset types
    default to 'ground', matching the simulation's original behaviour.
    """
    return "fixedwing" if mav_type in _FIXEDWING_MAV_TYPES else "ground"


# Sanity bounds for the parameter-derived values below: a hybrid swarm's
# copters and fixedwings are independently configured and genuinely do
# cruise at different speeds/radii, so these are read from each
# vehicle's OWN parameters instead of one shared default -- but an
# implausible value (param not actually downloaded yet, wrong units,
# a copter that somehow reports a huge number) should fall back to the
# caller's own default rather than silently poisoning the sim with a
# bogus speed. These ranges are deliberately generous, not precise
# flight limits.
_MIN_PLAUSIBLE_SPEED_M_S = 1.0
_MAX_PLAUSIBLE_SPEED_M_S = 60.0
_MIN_PLAUSIBLE_RADIUS_M = 0.5
_MAX_PLAUSIBLE_RADIUS_M = 500.0


def _plausible(value, lo, hi):
    return value is not None and lo <= value <= hi


def vehicle_cruise_speed_m_s(vehicle, vtype):
    """This drone's own configured cruise speed (m/s), read from its
    parameters -- NOT a single hardcoded default for the whole swarm,
    since a hybrid swarm's copters and fixedwings are independently
    configured and genuinely cruise at different speeds.

    ArduCopter: WPNAV_SPEED -- horizontal waypoint speed target, in cm/s.
    ArduPlane (incl. QuadPlane/VTOL): TRIM_ARSPD_CM -- target cruise
    airspeed, ALSO in cm/s (ArduPilot forum/parameter docs confirm cm/s,
    e.g. 1500 = 15 m/s). ArduPlane 4.5 renamed this to AIRSPEED_CRUISE
    AND converted it to m/s (confirmed by the official 4.5 release
    notes -- part of a deliberate "centi-units -> real units" cleanup
    across plane parameters), so on 4.5+ firmware TRIM_ARSPD_CM genuinely
    doesn't exist at all; AIRSPEED_CRUISE is tried next and used as-is
    (no /100 -- it really is already m/s, unlike its predecessor).

    Returns None if neither parameter is present or the result is
    outside a sane range, so callers fall back to their own default
    instead of a bogus value.
    """
    try:
        if vtype == "fixedwing":
            raw = vehicle.parameters.get("TRIM_ARSPD_CM")
            if raw is not None:
                speed = float(raw) / 100.0
            else:
                raw = vehicle.parameters.get("AIRSPEED_CRUISE")
                speed = float(raw) if raw is not None else None
        else:
            raw = vehicle.parameters.get("WPNAV_SPEED")
            speed = float(raw) / 100.0 if raw is not None else None
    except Exception:
        return None
    return (
        speed
        if _plausible(speed, _MIN_PLAUSIBLE_SPEED_M_S, _MAX_PLAUSIBLE_SPEED_M_S)
        else None
    )


def vehicle_waypoint_radius_m(vehicle, vtype):
    """This drone's own configured waypoint-arrival radius (metres),
    read from its parameters:

    ArduPlane (incl. QuadPlane/VTOL): WP_LOITER_RAD -- the radius it
    orbits its final waypoint at (it can't hover, so it circles instead
    of freezing). Already in metres.
    ArduCopter: WPNAV_RADIUS -- how close it must get to a waypoint to
    consider it reached, in cm (converted to metres here). Copters
    never orbit (they can hover), so this becomes the bot's
    capture_radius instead of a loiter radius.

    Returns None if the parameter is missing or outside a sane range.
    """
    try:
        if vtype == "fixedwing":
            raw = vehicle.parameters.get("WP_LOITER_RAD")
            radius = abs(float(raw)) if raw is not None else None
        else:
            raw = vehicle.parameters.get("WPNAV_RADIUS")
            radius = float(raw) / 100.0 if raw is not None else None
    except Exception:
        return None
    return (
        radius
        if _plausible(radius, _MIN_PLAUSIBLE_RADIUS_M, _MAX_PLAUSIBLE_RADIUS_M)
        else None
    )


_MIN_PLAUSIBLE_BANK_DEG = 5.0
_MAX_PLAUSIBLE_BANK_DEG = 75.0


def vehicle_bank_angle_deg(vehicle, vtype):
    """This fixedwing/quadplane drone's own configured max bank angle
    (degrees), read from its parameters -- NOT the single
    Variables.bank_angle_deg applied to every fixedwing bot by default.

    A bot that assumes a steeper bank than the real aircraft can
    actually fly turns tighter/faster than the real aircraft can
    physically match -- confirmed by observation: the bot pulling
    ahead of its drone specifically during turns (not on straight
    legs, where speed alone governs it -- see
    vehicle_cruise_speed_m_s()).

    ArduPlane: ROLL_LIMIT_DEG (current firmware, already degrees) if
    present, else the legacy LIM_ROLL_CD (centidegrees, /100). Not
    applicable to copters (vtype != 'fixedwing' returns None).

    Returns None if neither parameter is available or the result is
    outside a sane range, so callers fall back to Variables.bank_angle_deg.
    """
    if vtype != "fixedwing":
        return None
    try:
        raw = vehicle.parameters.get("ROLL_LIMIT_DEG")
        if raw is not None:
            bank = float(raw)
        else:
            raw = vehicle.parameters.get("LIM_ROLL_CD")
            bank = float(raw) / 100.0 if raw is not None else None
    except Exception:
        return None
    return (
        bank
        if _plausible(bank, _MIN_PLAUSIBLE_BANK_DEG, _MAX_PLAUSIBLE_BANK_DEG)
        else None
    )


def drone_system_id(vehicle):
    """MAVLink system id of a connected dronekit vehicle."""
    return int(vehicle._master.target_system)


class Vehicles:
    def __init__(self, conn_str, heartbeat_timeout=3, max_workers=None):
        self.conn_str = conn_str
        self.heartbeat_timeout = heartbeat_timeout
        self.max_workers = max_workers or len(conn_str)
        self.drones = [None] * len(conn_str)
        # Auto-detected per-drone sim vehicle_type, index-aligned with
        # self.drones -- see mav_type_to_vehicle_type() above.
        self.vehicle_types = [None] * len(conn_str)
        # Each drone's own configured cruise speed (m/s), waypoint-
        # arrival radius (m), and -- fixedwing/quadplane only -- max
        # bank angle (degrees), read from its parameters -- see
        # vehicle_cruise_speed_m_s()/vehicle_waypoint_radius_m()/
        # vehicle_bank_angle_deg() above. None where the drone's
        # parameters didn't yield a plausible value; callers fall back
        # to their own default in that case.
        self.speeds = [None] * len(conn_str)
        self.radii = [None] * len(conn_str)
        self.bank_angles = [None] * len(conn_str)
        self.system_ids = []
        self.connect_all()

    def get_positions(self, geoToCart, origin, endDistance):
        return [
            (
                *[
                    coord
                    for coord in geoToCart(
                        origin,
                        endDistance,
                        [
                            drone.location.global_relative_frame.lat,
                            drone.location.global_relative_frame.lon,
                        ],
                    )
                ],
                compass_to_math_rad(drone.heading),
            )
            for drone in self.drones
        ]

    def connect_all(self):
        with ThreadPoolExecutor(max_workers=self.max_workers) as executor:
            futures = {
                executor.submit(self._connect_one, i, s): i
                for i, s in enumerate(self.conn_str)
            }
            for future in as_completed(futures):
                future.result()  # re-raises any unhandled exception, if you want strictness

        # Drop failed slots (None) from both lists together, so
        # vehicle_types stays index-aligned with drones. Without this,
        # get_positions() and Loiter_param() below iterate every slot
        # unconditionally and crash with AttributeError the moment any
        # one of these ports fails to connect -- which takes down
        # Swarm's search()/goal()/AutosplitMission() entirely, not just
        # the one missing drone.
        #
        # NOTE: this renumbers bot ids relative to connection order, not
        # the original port list -- e.g. if the drone on port 14553
        # (the 3rd configured port) fails, the drone on 14554 becomes
        # bot 3, not bot 4. There's no stable port->id mapping anywhere
        # else in this codebase (remove_bot/add_bot/goal all address
        # bots by plain position in self.drones), so this matches how
        # the rest of the system already treats len(self.drones) as
        # the swarm size -- main.py passes it straight through as
        # num_bots.
        #
        # Bots are then ordered by the drone's MAVLink system id, so bot
        # position N-1 is the Nth lowest system id (which IS system id N
        # for the usual 1..N numbering) no matter which port each drone
        # happened to connect on. system_ids stays index-aligned with
        # drones; the sysid<->bot-id translation lives in bot_id_for_sysid()
        # / sysid_for_bot_id() below.
        paired = [
            (drone, vtype, speed, radius, bank)
            for drone, vtype, speed, radius, bank in zip(
                self.drones,
                self.vehicle_types,
                self.speeds,
                self.radii,
                self.bank_angles,
            )
            if drone is not None
        ]
        paired.sort(key=lambda p: drone_system_id(p[0]))
        self.drones = [drone for drone, _, _, _, _ in paired]
        self.vehicle_types = [vtype for _, vtype, _, _, _ in paired]
        self.speeds = [speed for _, _, speed, _, _ in paired]
        self.radii = [radius for _, _, _, radius, _ in paired]
        self.bank_angles = [bank for _, _, _, _, bank in paired]
        self.system_ids = [drone_system_id(drone) for drone in self.drones]

        connected = len(self.drones)
        print(f"Number of Drones Connected: {connected}/{len(self.conn_str)}")

    def _connect_one(self, index, conn_string):
        try:
            vehicle = connect(
                conn_string, heartbeat_timeout=self.heartbeat_timeout, source_system=253
            )

            self.drones[index] = vehicle
            vtype = mav_type_to_vehicle_type(vehicle._vehicle_type)
            self.vehicle_types[index] = vtype
            speed = vehicle_cruise_speed_m_s(vehicle, vtype)
            radius = vehicle_waypoint_radius_m(vehicle, vtype)
            bank = vehicle_bank_angle_deg(vehicle, vtype)
            self.speeds[index] = speed
            self.radii[index] = radius
            self.bank_angles[index] = bank
            print(
                f"Drone {index} connected ({conn_string}) -- "
                f"detected vehicle_type={vtype} speed={speed} radius={radius} "
                f"bank_angle_deg={bank}"
            )
        except APIException:
            print(f"Drone {index} failed: no heartbeat ({conn_string})")
        except Exception as e:
            print(f"Drone {index} failed: {e} ({conn_string})")

    def bot_id_for_sysid(self, sysid):
        """Swarm bot id (1-based position) of the drone with this MAVLink
        system id. Raises ValueError if no connected drone has it."""
        try:
            return self.system_ids.index(int(sysid)) + 1
        except ValueError:
            raise ValueError(f"no connected drone with system id {sysid}") from None

    def sysid_for_bot_id(self, bot_id):
        """MAVLink system id of the drone at this swarm bot id."""
        return self.system_ids[int(bot_id) - 1]

    def Loiter_param(self):
        """Largest WP_LOITER_RAD among the connected drones that actually
        have it.

        WP_LOITER_RAD is an ArduPlane-only parameter -- ArduCopter has no
        such parameter at all, so indexing vehicle.parameters["WP_LOITER_RAD"]
        on a copter raises KeyError (dronekit's Parameters.__getitem__
        looks it up in the vehicle's own reported param set). In a hybrid
        swarm that's a real combination now, not a hypothetical one, so
        this uses .get() (falls back to the dict-like default, None,
        instead of raising) and filters those Nones out before taking
        the max -- an all-copter swarm correctly returns None, same as
        before when nothing had the parameter at all.
        """
        radii = [
            r
            for r in (drone.parameters.get("WP_LOITER_RAD") for drone in self.drones)
            if r is not None
        ]
        return max(radii, default=None)
