import numpy as np

from swarm_tasks.utils.latlon import LatLonConversion

DEFAULT_NEIGHBOURHOOD_VAL = 6  # neighbourhood radius
DEFAULT_SIZE = 0.4  # Radius of chassis
MAX_SPEED = 1.5
MAX_ANGULAR = 0.3
DEFAULT_STATE = 0
DEFAULT_MIN_SPEED = 0.6  # Stall-speed floor for fixed-wing kinematics
DEFAULT_VEHICLE_TYPE = "ground"  #'ground' or 'fixedwing'

GRAVITY = 9.81
DEFAULT_BANK_ANGLE_DEG = 30  # Max bank angle for fixedwing turns
DEFAULT_FIXEDWING_SPEED = 20  # m/s cruise airspeed


def turn_rate_from_bank(bank_angle_deg, speed, g=GRAVITY):
    """
    Coordinated-turn relation: max sustainable turn rate (rad/s)
    for a given bank angle and airspeed.
            omega_max = g * tan(bank) / v
    """
    bank_rad = np.radians(bank_angle_deg)
    return g * np.tan(bank_rad) / max(speed, 1e-3)


def turn_radius_from_bank(bank_angle_deg, speed, g=GRAVITY):
    """
    Minimum turn radius (m) achievable at a given bank angle and airspeed.
            R_min = v^2 / (g * tan(bank))
    """
    bank_rad = np.radians(bank_angle_deg)
    return (speed**2) / (g * np.tan(bank_rad))


class Bot:
    """
    Robot class
    """

    def __init__(
        self,
        x,
        y,
        theta=0,
        state=DEFAULT_STATE,
        size=DEFAULT_SIZE,
        neighbourhood_radius=DEFAULT_NEIGHBOURHOOD_VAL,
        speed=MAX_SPEED,
        max_turn_speed=MAX_ANGULAR,
        min_speed=DEFAULT_MIN_SPEED,
        vehicle_type=DEFAULT_VEHICLE_TYPE,
        bank_angle_deg=None,
        verbose=False,
    ):

        # MAVLink system id of the drone this bot stands for. Set by Swarm
        # after the bot is built (None for a bot with no real drone behind it).
        self.sysid = None

        # Basic attributes
        self.x = x
        self.y = y
        self.theta = theta
        self.state = state
        self.size = size
        self.neighbourhood_radius = neighbourhood_radius
        self.max_speed = speed
        self.min_speed = min_speed  # Stall speed: fixedwing bots never go below this
        self.vehicle_type = vehicle_type  #'ground': can pivot in place. 'fixedwing': can't stop or pivot
        self.verbose = verbose

        # Turn rate: for fixedwing, derive from bank angle + speed if given
        # (bank_angle_deg takes priority over an explicit max_turn_speed,
        # since it's the physically meaningful parameter for a real aircraft)
        self.bank_angle_deg = bank_angle_deg
        if self.vehicle_type == "fixedwing" and bank_angle_deg is not None:
            self.max_turn_speed = turn_rate_from_bank(bank_angle_deg, speed)  # rad/s
        else:
            self.max_turn_speed = max_turn_speed

        if verbose:
            print("New bot spawned at: (" + str(x) + "," + str(y) + ")")

        self.sim = None  # Reference to Simulation object that spawns the bot

        # Goal (weight given to goal in potential field)
        self.goal_given = False
        self.goal = [0, 0]
        self.step_size = 0.1

        # Ordered queue of waypoints still to visit *after* the current
        # goal, e.g. loaded from a CSV via utils.waypoints.load_goals_csv()
        # and handed in with set_goal_sequence(). advance_goal() consumes
        # this one at a time as each goal is captured -- see that method,
        # and controllers/guided.py's freeze_on_arrival option, which
        # calls it automatically.
        self.goal_queue = []

        # Sticky "stop" flag: True once this bot has been frozen after
        # reaching a goal (see stop_if_reached()), OR before it's ever
        # been given one. reached_goal() is a live distance check that
        # can flicker True/False as the bot orbits or overshoots;
        # `done` is the one-shot latch that actually halts computation/
        # movement for this bot specifically, while the rest of the
        # swarm keeps going. Cmd.exec() (see controllers/command.py)
        # checks this and no-ops when True, so every existing task/
        # example loop gets the freeze for free just by continuing to
        # call cmd.exec(bot) each frame. set_goal() clears it
        # automatically, so assigning a new goal always wakes the bot
        # back up.
        #
        # Starts True (not False): a bot with goal_exists()==False
        # falls through line_waypoint_guidance()'s very first check to
        # `Cmd(dir_=bot.theta, speed=bot.max_speed)` -- a NONZERO
        # speed, same heading it already had -- and unless `done` is
        # already True, Cmd.exec() isn't a no-op, so that command
        # actually gets applied: a bot that was never assigned a goal
        # at all would otherwise fly straight ahead at full speed
        # forever (confirmed directly: an un-goaled bot advanced every
        # single tick, no different from one mid-route). This matters
        # beyond just the sim -- swarm.py's per-tick loops call
        # send_reposition() right along with it, so a real connected
        # vehicle that a mission command didn't happen to select (e.g.
        # more vehicles connected than effective_num_uavs/
        # selected_uav_ids covered) would be actively flown via
        # MAV_CMD_DO_REPOSITION toward an ever-receding point in
        # whatever direction it happened to be facing at populate time
        # -- with nothing ever telling it to stop, since freezing only
        # ever happens on ARRIVAL and a bot with no goal can never
        # arrive anywhere. Starting frozen instead means a bot only
        # ever moves once something has actually given it somewhere to
        # go -- set_goal()/set_goal_sequence() still immediately un-
        # freeze it exactly as before.
        self.done = True

        # Loiter state (used by guided waypoint navigation for fixedwing bots,
        # see controllers/guided.py's _orbit_cmd()/loiter_radius handling).
        self.loitering = False
        self.loiter_center = None

        # Cached path (e.g. a curvature-constrained Bezier curve to the goal)
        self.path = None
        self.path_goal = (
            None  # Goal the cached path was generated for; regenerate if goal changes
        )

        # Straight-line leg tracking (controllers/guided.py: line_waypoint_guidance).
        # line_start is wherever the bot was when the CURRENT goal was set --
        # i.e. the other end of the line segment being flown -- and
        # line_start_goal is which goal that was captured for, mirroring
        # path/path_goal's "only refresh when the goal actually changes" pattern.
        self.line_start = None
        self.line_start_goal = None

        # Set by line_waypoint_guidance() (controllers/guided.py) to the
        # ACTUAL point it's steering at this tick -- bot.goal offset by
        # avoidance, when avoid_bots is on -- not just the raw goal. A
        # caller driving real hardware should send THIS to the vehicle
        # (see hardware/ardupilot_link.py), not bot.goal, or the real
        # vehicle never benefits from avoidance at all. None until the
        # guidance function has run at least once.
        self.last_guidance_target = None

        # Set by Simulation when constructed with connection_strings --
        # an ArduPilotLink for this bot's real vehicle, or None in pure sim
        self.ardupilot = None

        # Optional earth-geometry check: independent of ardupilot above --
        # works for a pure-sim bot with no real vehicle attached at all.
        # Set via set_geo_check(converter) with a LatLonConversion anchored
        # to wherever bot (0,0) should sit on the real Earth. When set,
        # every step() call also checks its own displacement against real
        # geodesic distance at the bot's actual (converted) location and
        # records the result in self.last_geo_check -- see step() and
        # set_geo_check() below for details. None (the default) disables
        # this entirely, at zero extra cost per step.
        self.geo_converter = None
        self.last_geo_check = None

        # For exploration
        self.explore_dir = np.pi * (2 * np.random.rand() - 1)

    def set_geo_check(self, geo_converter):
        """
        Attaches a LatLonConversion (see utils/latlon.py) to this bot so
        that every subsequent step() call also verifies its own x,y
        displacement against real Earth geometry (great-circle distance)
        at the bot's actual location -- not just near the converter's
        origin.

        This is independent of hardware/ardupilot_link.py's ArduPilotLink:
        it works for a bot that's never touched real hardware, purely to
        answer "if this bot's (0,0) were really at lat0,lon0, how far off
        from true 1-unit-per-metre is its movement turning out to be at
        wherever it currently is in the world?"

        Args:
                geo_converter: a LatLonConversion instance (or None to
                        detach/disable the check again).
        """
        self.geo_converter = geo_converter
        self.last_geo_check = None

    def turn_radius(self):
        """
        Minimum turn radius (m) at this bot's current speed and bank angle.
        Only meaningful for fixedwing bots with bank_angle_deg set.
        """
        if self.bank_angle_deg is None:
            return None
        return turn_radius_from_bank(self.bank_angle_deg, self.max_speed)

    def reached_goal(self, capture_radius=5.0):
        """
        True if a goal is set and the bot's current position is within
        capture_radius of it. Works regardless of whether bot.x/y are
        being driven by the sim's own kinematics (move()) or synced in
        from a real vehicle's telemetry (see hardware/ardupilot_link.py) --
        it only looks at position, not at how it got there.

        This is a live check, not a latch -- e.g. a fixedwing bot that
        can't stop may drift back outside capture_radius after arriving
        and this will flip back to False. Use stop_if_reached() if you
        actually want the bot to halt (freeze) the first time it arrives
        and stay that way.
        """
        if not self.goal_given:
            return False
        return self.dist(*self.goal) < capture_radius

    def stop_if_reached(self, capture_radius=5.0):
        """
        Freezes the bot the moment it reaches its goal, and keeps it
        frozen from then on -- unlike reached_goal(), which just reports
        live distance and can flip back to False (e.g. a fixedwing bot
        drifting past its capture radius).

        Once frozen (self.done == True):
          - Cmd.exec() becomes a no-op for this bot, so its position/
            heading simply hold at wherever it was when captured. No
            further motion is applied to it, even though the rest of
            the swarm keeps moving normally.
          - Callers can also skip computing guidance for this bot
            entirely (`if bot.done: continue`) to stop the per-frame
            computation for it too, not just the motion -- see
            controllers/guided.py's freeze_on_arrival option, which sets
            this automatically.

        The bot un-freezes automatically the next time set_goal() (or
        resume()) is called on it.

        Returns: True if the bot is now (or was already) frozen.
        """
        if self.done:
            return True
        if self.reached_goal(capture_radius):
            self.done = True
        return self.done

    def resume(self):
        """Un-freezes the bot (clears self.done) without changing its goal."""
        self.done = False

    def dist(self, x, y):
        """
        Returns the distance of the centre of the robot
        from the point x,y
        """
        return np.linalg.norm([x - self.x, y - self.y])

    def dist_v(self, x, y):
        angle_rad = np.radians(self.theta % 360)
        forward = np.array([np.cos(angle_rad), np.sin(angle_rad)])

        vec = np.array([self.x - x, self.y - y])

        dot = np.dot(forward, vec)
        actual_distance = np.linalg.norm(vec)
        return dot > 0, actual_distance

    def neighbours(self, radius=None, single_state=False, state=None):
        """
        Returns: List of neighbours
        Args:
                radius: Neighbourhood radius to use (default: self.neighbourhood_radius)
                single_state: If true, then return only neighbours of the given state

        (TODO) Enhancement:
        (2) Include self
        """
        if radius == None:
            radius = self.neighbourhood_radius

        neighbours = []
        if self.sim == None:
            print("WARNING: Robot is not linked to a simulation")
            return neighbours

        for bot in self.sim.swarm:
            # Skip if a single state is needed
            if single_state:
                if bot.state != state:
                    continue

            # Get distance to bot
            d = bot.dist(self.x, self.y)

            # Check if neighbour
            if d > 0 and d < radius:
                neighbours.append(bot)

        return neighbours

    def step(self, distance):
        x_ = self.x + distance * np.cos(self.theta)
        y_ = self.y + distance * np.sin(self.theta)

        if self.sim.check_free(
            x_, y_, self.size + 0.01, ignore=self
        ) or not self.sim.check_free(self.x, self.y, self.size - 0.01, ignore=self):
            # First condition checks if new location is free
            # Second condition checks for unstuck (current location not free)
            x_prev, y_prev = self.x, self.y
            self.x, self.y = x_, y_

            if self.geo_converter is not None:
                # Verify this step's displacement against real Earth
                # geometry at the bot's ACTUAL current location -- see
                # LatLonConversion.verify_xy_movement() in utils/latlon.py.
                # Cheap (a couple of interp1d calls + one haversine), and
                # only runs at all if set_geo_check() was called, so a
                # plain sim run with no geo-check attached pays nothing
                # extra here.
                self.last_geo_check = self.geo_converter.verify_xy_movement(
                    (x_prev, y_prev), (self.x, self.y)
                )
        else:
            pass
            # print("Collision!")

    def turn(self, angle):
        self.theta += angle
        while self.theta > 2 * np.pi:
            self.theta -= 2 * np.pi
        while self.theta < 0:
            self.theta += 2 * np.pi

    def move(self, direction, speed):
        """
        The robot moves one step in the given direction
        The size of step os step_size*speed
        The speed used is the minimum of speed param and self.max_speed

        'ground'/copter bots: omnidirectional, like a real multirotor --
        it doesn't need to face its direction of travel before it can
        move there (already true of movement, below) and its YAW isn't
        bank-angle-limited the way a fixed-wing's heading is, so theta
        snaps directly to `direction` every tick instead of being rate-
        limited. Always translates forward at a speed clamped to
        [0, max_speed] (it CAN slow to a stop/hover, unlike a
        fixedwing).

        'fixedwing' bots: cannot stop or pivot in place. Every step it
        translates forward at a speed clamped to [min_speed, max_speed],
        while the heading turns toward `direction` at a rate bounded by
        max_turn_speed (a Dubins-car / bank-limited-turn approximation of
        fixed-wing flight) -- a real physical constraint, unlike a
        copter's heading.

        An earlier version rate-limited theta for ground/copter bots
        too, "purely for a readable heading arrow" -- but a real copter
        doesn't carve a turn radius between legs the way a fixed-wing
        does (WPNAV_SPEED's cornering is jerk-limited deceleration/
        acceleration, not a geometric arc), and even though that rate
        limit never gated movement (fixed separately, see below), the
        heading arrow still visibly swung through what looked like a
        turn over several seconds -- visible on the plot as the copter
        bot appearing to arc between waypoints instead of cornering
        sharply like its real vehicle does.

        A version before THAT pivoted in place (skipped translating
        entirely) whenever the needed heading correction was large,
        using a buggy threshold that compared the correction (an angle)
        directly against max_turn_speed (a rate, rad/s) with no
        *step_size -- so it pivoted by up to a full max_turn_speed
        radians in ONE tick (far more than the configured rate actually
        allows) and moved 0m that tick. Harmless for a bot flying alone
        (it reaches the right heading in 1-2 ticks either way), but
        under avoid_bots repulsion from a nearby bot the commanded
        direction itself shifts by a similar amount tick to tick --
        confirmed by reproducing it directly: a bot stuck re-aiming at
        a moving target for 259 of 300 ticks, 0m net progress toward
        its goal. That's what showed up as a bot "loitering" near one
        point instead of advancing to the next.
        """
        turn_angle = direction - self.theta
        while turn_angle >= np.pi:
            turn_angle -= 2 * np.pi
        while turn_angle <= -np.pi:
            turn_angle += 2 * np.pi

        if self.vehicle_type == "fixedwing":
            max_turn_this_step = self.max_turn_speed * self.step_size
            turn_step = np.clip(turn_angle, -max_turn_this_step, max_turn_this_step)
            self.turn(turn_step)
            # Cannot stop/hover: speed is clamped to [min_speed, max_speed]
            cmd_speed = np.clip(speed, self.min_speed, self.max_speed)
        else:
            self.turn(turn_angle)
            # Can slow to a stop/hover, but never commanded backwards
            cmd_speed = np.clip(speed, 0.0, self.max_speed)
        self.step(cmd_speed * self.step_size)

    def unstuck(self, r):
        """
        Use only if no other option is available
        """
        x, y = np.random.randn(2) * r
        while not self.sim.check_free(x, y, r, ignore=self):
            x, y = np.random.randn(2) * r
        self.x, self.y = x, y

    def set_sim(self, sim):
        self.sim = sim

    def set_goal(self, x, y):
        self.goal = (x, y)
        self.goal_given = True
        self.loitering = (
            False  # New waypoint: fly direct, not loiter, until captured again
        )
        self.loiter_center = None
        self.path = None  # Force regeneration of any cached path
        self.path_goal = None
        self.line_start = (
            None  # Force line_waypoint_guidance to re-anchor the new leg's start
        )
        self.line_start_goal = None
        self.done = False  # New goal: un-freeze, even if the previous one had captured
        # self.state=?
        # NOTE: does not touch self.goal_queue -- advance_goal() relies on
        # calling this to move to an already-popped next waypoint without
        # it wiping the rest of the queue. Call clear_goal_queue()
        # explicitly if you want a one-off set_goal() to also drop any
        # queued sequence.

    def set_goal_sequence(self, waypoints):
        """
        Gives the bot a whole ordered list of goals to fly through, e.g.
        loaded from a CSV via utils.waypoints.load_goals_csv(). The
        first waypoint is set immediately as the current goal; the rest
        are queued and consumed automatically -- one at a time, via
        advance_goal() -- as each prior goal is captured.

        Typically you won't call advance_goal() yourself: pass
        freeze_on_arrival=True to controllers.guided's guidance
        functions and they'll call it for you every time a goal is
        reached, so the bot flies its whole list of goals in order and
        then freezes once the list is exhausted, with no extra logic
        needed in the calling loop.

        Args:
                waypoints: a non-empty iterable of (x, y) pairs, in the
                        order they should be visited.
        """
        waypoints = list(waypoints)
        if not waypoints:
            raise ValueError("waypoints must be a non-empty list of (x, y) pairs")

        self.goal_queue = waypoints[1:]
        self.set_goal(*waypoints[0])

    def advance_goal(self):
        """
        Moves on to the next queued waypoint (from set_goal_sequence()),
        or freezes the bot if the queue is empty -- there's nowhere left
        to go. Meant to be called once a goal has just been captured;
        see controllers.guided's freeze_on_arrival option, which calls
        this automatically instead of unconditionally freezing on
        arrival, so a bot with a queued sequence keeps moving through it
        and only freezes after the *last* one.

        Returns: True if the bot is now frozen (queue was empty), else
                False (bot.goal has been updated to the next waypoint and
                bot.done was cleared, so it's flying again).
        """
        if self.goal_queue:
            nxt = self.goal_queue.pop(0)
            self.set_goal(*nxt)
            print(f"robot goal set to {nxt} bot id: {self.sysid}")
            return False
        self.done = True
        return True

    def clear_goal_queue(self):
        """Drops any remaining queued waypoints without touching the current goal."""
        self.goal_queue = []

    def cancel_goal(self):
        self.goal_given = False
        self.loitering = False
        self.loiter_center = None
        self.path = None
        self.path_goal = None
        self.done = False  # No goal left to be "done" with
        self.goal_queue = []  # Nothing left to advance to, either

    def goal_exists(self):
        return self.goal_given

    def get_state(self):
        return self.state

    def set_state(self, state):
        self.state = state

    def get_pose(self):
        return self.x, self.y, self.theta

    def get_position(self):
        return self.x, self.y

    def get_dir(self):
        return self.theta
