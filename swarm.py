import os, sys

sys.path.insert(
    0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
)
from swarm_tasks.simulation import simulation as sim
from swarm_tasks.mission.bezier_curve import BezierCurveMultiple, check_coverage_area
from swarm_tasks.controllers.guided import line_waypoint_guidance
from swarm_tasks.utils import waypoints as wp
from swarm_tasks.Thread import Thread
from time import sleep
import threading
import traceback
from swarm_tasks.utils.locatePosition import geoToCart, cartToGeo
from utils import compute_lookahead_target, send_reposition, compass_to_math_rad
from time import time
import random
from swarm_tasks.mission.groupsplitauto import AutoSplitMission
from swarm_tasks.mission.groupsplitspecific import SpecificSplitMission
from swarm_tasks.telemetry_log import setup_swarm_logger
from math import sin, cos, pi


class _NullAxes:
    """Stand-in for Gui.ax when the GUI is off -- search()/goal()/
    AutosplitMission()/SpecificSplit() plot waypoint markers straight
    onto self.gui.ax without going through a method, so the no-op has
    to live here too."""

    def plot(self, *args, **kwargs):
        return (None,)


class _NullGui:
    """Stand-in for visualizer.Gui when Variables.gui is False -- this
    is a headless backend service (no plot is ever shown), so the
    default run should neither import matplotlib/Qt nor pay the cost of
    opening a figure window just to never draw into it.

    Implements every method swarm.py calls on self.gui as a no-op, so
    none of those call sites need an `if self.gui is not None` guard --
    there's always a valid object here, real or null.
    """

    ax = _NullAxes()

    def show_env(self):
        pass

    def show_bots(self):
        pass

    def show_goals(self, *args, **kwargs):
        pass

    def show_route(self, *args, **kwargs):
        pass

    def clear_routes(self, *args, **kwargs):
        pass

    def show_gps_positions(self, *args, **kwargs):
        pass

    def show_lookahead_targets(self, *args, **kwargs):
        pass

    def update(self):
        pass

    def run(self):
        pass

    def close(self):
        pass


class Swarm:
    def __init__(self, num_bots, Variables, vehicle):
        self.num_bots = num_bots
        self.Variables = Variables
        self.vehicle = vehicle
        # Persistent per-run log (~/Documents/swarm_env/logs/<run_id>.log)
        # of every bot's guidance state, tick by tick -- see
        # telemetry_log.py. Always on, independent of debug_reposition
        # (console-only). This is what to check when a live run looks
        # wrong and the cause isn't obvious from the plot alone.
        self.log = setup_swarm_logger(self.Variables.run_id)
        # Hybrid swarm: each bot gets the kinematics matching its OWN
        # connected drone (vehicle.vehicle_types, auto-detected per-drone
        # from its MAVLink HEARTBEAT in vehicle.py) instead of forcing
        # every bot to 'fixedwing', so quadcopters and fixed-wing planes
        # can fly in the same swarm correctly.
        self.s = sim.Simulation(
            vehicle_type=vehicle.vehicle_types,
            world_size=self.Variables.world_size,
            speed=self._resolve_bot_speeds(),
            bank_angle_deg=self._resolve_bot_bank_angles(),
        )
        self.gui = self._make_gui()
        self.current_function = {
            "goal": [False, None],
            "search": [False, None],
            "spilt": [False, None],
        }
        self.isToStop = False
        # Goal jobs run CONCURRENTLY -- one per assignment, each owning a
        # set of bot ids and its own stop Event, so stopping the job for
        # bot 1 leaves the job for bot 2 flying. current_function["goal"]
        # mirrors this list so running_funcs()/stopAll() still work.
        self.goal_jobs = []
        # Self-healing: bots taken out of service by remove_bot(), the
        # work they had left over, and what each bot was originally
        # given (so "covered 5 of 8" can be reported).
        self.removed_bots = set()
        self.orphan_work = []  # FIFO of {"from": id, "points": [...]}
        self.assigned_seq = {}  # swarm index -> the list it was handed
        self._heal_lock = threading.Lock()
        # Workers run on background threads; matplotlib/Qt calls are not
        # thread-safe, so they only publish state here and the main
        # thread's gui_tick() does the actual drawing.
        #
        # Per-bot render state, keyed by bot index:
        #     index -> (goal, (rx, ry), (tx, ty))
        # A dict rather than a single-slot Queue of whole-swarm frames:
        # each concurrent job only knows about its own bots, so one
        # worker's full snapshot would blank out every bot the other
        # jobs own. Each merges only its own slice in, under the lock.
        self._gui_state = {}
        self._gui_lock = threading.Lock()
        self._gui_close_requested = False
        self.check_in_time = 0
        # Per-bot timestamp of the last send_reposition() call, so the
        # tick loop (runs every tick_interval_s, e.g. 0.1s) can still
        # gate the actual MAVLink command out to reposition_interval_s
        # instead of spamming one every tick.
        self._last_reposition_time = {}
        # Throttle for the debug line (Variables.debug_reposition).
        self._last_debug_time = {}
        self._last_idle_pump = 0.0

    def _resolve_bot_speeds(self):
        """Each bot's speed -- always taken from the web-config-panel
        fields Variables.speed (fixedwing) / Variables.copter_speed
        (ground/copter), by request, even for a drone whose own real
        vehicle reported a detected cruise speed (vehicle.py's
        per-drone WPNAV_SPEED/TRIM_ARSPD_CM detection -- still used
        elsewhere, e.g. loiter/bank-angle, just not for this). Reads
        Variables.speed/copter_speed FRESH every call (not cached), so
        a web-config-panel edit is picked up the next time this is
        called -- see _populate_sim(). This only changes the
        SIMULATED bot's kinematics -- the real vehicle's own flight
        speed is untouched (send_reposition() never sends a
        groundspeed).
        """
        return [
            self.Variables.copter_speed if vtype != "fixedwing" else self.Variables.speed
            for vtype in self.vehicle.vehicle_types
        ]

    def _resolve_bot_bank_angles(self):
        """Each fixedwing bot's own real configured max bank angle
        (vehicle.py's ROLL_LIMIT_DEG/LIM_ROLL_CD detection), falling
        back to Variables.bank_angle_deg only when a drone's own
        parameters didn't yield one. A bot that assumes a steeper bank
        than its real aircraft can actually fly turns tighter/faster
        than the real aircraft can match -- confirmed by observation:
        the bot pulling ahead of its drone specifically during turns.
        Ground/copter bots don't use bank angle at all (robot.py only
        reads it for vehicle_type == 'fixedwing'), so their entry here
        is irrelevant and left as the plain Variables default.
        """
        return [
            bank if bank is not None else self.Variables.bank_angle_deg
            for bank in self.vehicle.bank_angles
        ]

    def _populate_sim(self, positions, num_bots):
        """Wraps Simulation._populate(): refreshes the sim's speed and
        bank_angle_deg from Variables immediately before rebuilding
        every bot, so editing speed/copter_speed/bank_angle_deg in the
        web config panel takes effect on the very next mission command
        (search/goal/split/specificsplit) -- _populate() already
        destroys and recreates every bot from scratch each time it
        runs, so there's nothing stale left over to carry a change
        forward; the previous behaviour of requiring a full process
        restart only existed because the sim's speed/bank_angle_deg
        were set once at Simulation construction and never refreshed.
        world_size still needs a restart: changing it means resizing
        the world/coverage-grid themselves, not just re-tagging bots.
        """
        self.s.speed = self._resolve_bot_speeds()
        self.s.bank_angle_deg = self._resolve_bot_bank_angles()
        self.s._populate(positions=positions, num_bots=num_bots)
        # Tag every freshly built bot with its drone's MAVLink system id.
        # Bots are rebuilt on each mission command, so this has to be
        # redone here rather than once at construction.
        for bot, sysid in zip(self.s.swarm, self.vehicle.system_ids):
            bot.sysid = sysid

    def stopAction(self):
        """call_back_action handed to Thread.stop(): the list of things
        this Swarm owns that must wind down.

        SIGNAL ONLY -- it must not block. Thread.stop() runs this
        BEFORE setting its stop event, and Thread.run() re-invokes the
        target whenever it returns while that event is still clear. So
        waiting for the worker here would deadlock: the worker would
        exit, run() would restart it, and we'd never get to the
        set(). Setting isToStop breaks searchSub's own `while True`,
        the worker returns, Thread.stop() then sets the event, and the
        join() in current_process() actually completes.

        The real teardown (closing the GUI, clearing per-run state)
        happens in teardown(), called after that join -- see
        current_process() in utils.py.
        """
        self.isToStop = True

    def teardown(self):
        """Release everything the just-stopped function owned, so the
        next one starts from a clean slate.

        Must run on the MAIN thread and only AFTER the worker thread
        has been joined -- current_process() does both. Leaves the plot
        open but drops what it was drawing, clears the per-run timers,
        and marks every function slot as no-longer-running so
        running_func() reports idle.
        """
        # The plot stays OPEN and pumping after a stop, so the last
        # state can still be looked at -- only what it draws is cleared.
        # The next run's _reset_gui() closes and rebuilds it.

        # Drop stale render state, so it can't be drawn onto the next
        # run's fresh axes.
        with self._gui_lock:
            self._gui_state.clear()
        if self.gui is not None:
            self.gui.clear_routes()
        self.goal_jobs = []
        # NOT removed_bots: taking a bot out of the swarm is a
        # membership fact that has to outlive a stop, or the next
        # search would silently fly it again. add_bot() puts it back.
        self.orphan_work = []
        self.assigned_seq.clear()
        # Cancel every bot's leftover goals -- the run they belonged to
        # is over. Without this a remove_bot() AFTER a stop harvests the
        # dead run's queue into orphan_work and the next run flies it;
        # that is where "covered 0/0 points, 14 left" came from.
        for bot in self.s.swarm:
            bot.cancel_goal()

        self.isToStop = False
        self._gui_close_requested = False
        self._last_reposition_time.clear()
        self._last_debug_time.clear()
        self.check_in_time = 0

        for key in self.current_function:
            self.current_function[key] = [False, None]

        print("Teardown complete: state reset, plot left open")

    def _make_gui(self):
        """Builds the object self.gui points at: a real visualizer.Gui
        when Variables.gui is on, or a _NullGui otherwise. matplotlib
        is only imported in the former case -- a no-GUI backend run
        never touches it at all.
        """
        if not self.Variables.gui:
            return _NullGui()
        from swarm_tasks.simulation import visualizer as viz

        return viz.Gui(self.s, isGui=True)

    def _reset_gui(self):
        """Close any existing plot and build a fresh Gui bound to the
        current simulation. A Gui caches per-run drawing state (trails,
        goal/gps/lookahead artist lists, coverage text) and its figure
        may already be closed, so reusing one across functions leaves
        the previous run's elements on the new axes -- rebuild instead.
        """
        if self.gui is not None:
            self.gui.close()
        self.gui = self._make_gui()

    def running_func(self):
        return next(
            ((k, v) for k, v in self.current_function.items() if v[0] is True), None
        )

    def running_funcs(self):
        """Every currently running function, not just the first one."""
        return [(k, v) for k, v in self.current_function.items() if v[0] is True]

    @staticmethod
    def _threads_of(slot):
        """A slot holds one Thread (search) or a list of them (goal) --
        normalise both to a list so callers don't have to care."""
        thread = slot[1]
        if thread is None:
            return []
        if isinstance(thread, (list, tuple)):
            return [t for t in thread if t is not None]
        return [thread]

    def _sync_goal_slot(self):
        """Mirror goal_jobs into current_function["goal"], so
        running_funcs(), stopAll() and current_process() keep working
        without knowing anything about per-bot jobs."""
        threads = [job["thread"] for job in self.goal_jobs]
        self.current_function["goal"] = [bool(threads), threads]

    @staticmethod
    def _avoid_sep_for_type(vtype):
        """Same type-based bot-bot separation _guide_and_reposition()
        uses for avoid_min_sep/avoid_radius -- pulled out here too so
        _formation_offsets() can size a goal-command formation ring
        against the exact same numbers, instead of duplicating (and
        risking drifting out of sync with) the constants."""
        return 100.0 if vtype == "fixedwing" else 15.0

    def _formation_offsets(self, owned):
        """Per-bot (dx, dy) spacing offset (metres) for a goal() command
        shared across multiple bot ids.

        Swarm.goal() hands every id in `owned` the SAME goal_latlon
        sequence -- harmless for a single bot, but for more than one it
        means they'd all be told to converge on the literal identical
        coordinate. avoid_bots' own avoid_min_sep then makes that
        geometrically impossible: closing in on one shared point
        necessarily puts them well inside min_sep of each other, so
        each bot's lookahead target keeps getting pushed away by the
        other two and they just orbit the shared point forever,
        never actually arriving -- confirmed as the cause of "copters
        stuck, can't reach the goal" when several share one point.

        No real multirotor/aircraft can occupy the same spot anyway,
        so this spreads them into a small ring around the shared
        point instead: every bot gets a fixed offset added to EVERY
        waypoint in its own copy of the route, so they fly the same
        path shape as a tight formation rather than literally stacked
        on each other. The ring's radius is sized so adjacent bots in
        it already clear every involved vehicle's own avoid_min_sep
        with margin, so the avoidance system never has to fight the
        formation itself to hold it.

        Returns {swarm_index: (dx, dy)}, all (0.0, 0.0) when `owned`
        has 0 or 1 bots -- nothing to spread apart.
        """
        n = len(owned)
        if n <= 1:
            return {i: (0.0, 0.0) for i in owned}
        max_sep = max(
            self._avoid_sep_for_type(
                self.vehicle.vehicle_types[i]
                if i < len(self.vehicle.vehicle_types)
                else "ground"
            )
            for i in owned
        )
        # Adjacent points on an n-point ring of radius R are
        # 2*R*sin(pi/n) apart -- solve for R so that clears max_sep
        # with a 30% margin.
        radius = 1.3 * max_sep / (2.0 * sin(pi / n))
        return {
            i: (radius * cos(2 * pi * k / n), radius * sin(2 * pi * k / n))
            for k, i in enumerate(owned)
        }

    def _assign(self, index, points):
        """Give a bot a goal list and remember it, so remove_bot() can
        say how much of it was covered before the bot dropped out."""
        points = [tuple(p) for p in points]
        self.s.swarm[index].set_goal_sequence(points)
        self.assigned_seq[index] = points

    @staticmethod
    def _remaining_points(bot, was_done=None):
        """What this bot still has to fly: the goal it is on (if it has
        not captured it yet) plus everything still queued behind it.

        was_done is the bot's done flag from BEFORE remove_bot() froze
        it -- reading bot.done here instead would drop the goal it was
        actively flying towards."""
        if was_done is None:
            was_done = bot.done
        points = []
        if bot.goal_exists() and not was_done and bot.goal is not None:
            points.append(tuple(bot.goal))
        points.extend(tuple(p) for p in bot.goal_queue)
        return points

    def remove_bot(self, bot_id, hand_over=True):
        """Take a bot out of service and leave its unflown points for
        whichever bot finishes its own work first.

        This is the self-heal entry point: pull bot 2 out after it has
        covered 5 of its 8 points and the remaining 3 go into
        orphan_work, to be claimed by the next bot that runs out of its
        own goals (see _claim_orphan_work). The removed bot is frozen
        and skipped by every worker loop until add_bot() brings it back.
        """
        bot_id = int(bot_id)
        index = bot_id - 1
        if not 0 <= index < len(self.s.swarm):
            print(f"remove_bot: no bot {bot_id}")
            return []
        bot = self.s.swarm[index]

        # Freeze BEFORE snapshotting the queue: _guide_and_reposition()
        # checks `if not b.done` before moving, so once this is set the
        # worker can no longer advance the queue out from under us and
        # make a point be handed over twice, or dropped.
        was_done = bot.done
        bot.done = True

        with self._heal_lock:
            if bot_id in self.removed_bots:
                print(f"Bot {bot_id} is already removed")
                bot.done = was_done
                return []
            self.removed_bots.add(bot_id)
            remaining = self._remaining_points(bot, was_done)
            assigned = self.assigned_seq.get(index, [])
            covered = max(0, len(assigned) - len(remaining))
            if hand_over and remaining:
                self.orphan_work.append({"from": bot_id, "points": remaining})

        # Drop its markers -- no worker commands it now.
        bot.clear_goal_queue()
        self._clear_state([index])
        print(
            f"Bot {bot_id} removed: covered {covered}/{len(assigned)} points, "
            f"{len(remaining)} left for the next free bot"
        )
        if not self.running_funcs():
            print(
                "  (nothing is running -- this only takes it out of the "
                "next search/goal; the plot has no live state to redraw)"
            )
        return remaining

    def add_bot(self, bot_id):
        """Put a removed bot back into service. It comes back idle, so
        the next tick of whichever job owns it will let it claim any
        orphaned work that is still waiting."""
        bot_id = int(bot_id)
        index = bot_id - 1
        if not 0 <= index < len(self.s.swarm):
            print(f"add_bot: no bot {bot_id}")
            return False
        with self._heal_lock:
            if bot_id not in self.removed_bots:
                print(f"Bot {bot_id} is already in the swarm")
                return False
            self.removed_bots.discard(bot_id)
        self.s.swarm[index].done = True  # idle => eligible to claim work
        print(f"Bot {bot_id} added back to the swarm")
        return True

    def _claim_orphan_work(self, index):
        """A bot that just ran out of goals takes over the oldest bundle
        left behind by a removed bot. Returns True if it picked one up.

        Called from the worker loops in place of a bare `if b.done:
        continue`, so the FIRST bot to finish gets the work -- which is
        exactly the hand-over rule: whoever is free first flies it.
        """
        with self._heal_lock:
            if not self.orphan_work or (index + 1) in self.removed_bots:
                return False
            work = self.orphan_work.pop(0)
        self._assign(index, work["points"])
        print(
            f"Self-heal: bot {index+1} took over {len(work['points'])} "
            f"points left by bot {work['from']}"
        )
        return True

    def _owned_indices(self):
        """Swarm indices currently commanded by SOME goal job."""
        return {i - 1 for job in self.goal_jobs for i in job["ids"]}

    def _publish_state(self, updates):
        """Merge one worker's slice of render state into the shared dict."""
        with self._gui_lock:
            self._gui_state.update(updates)

    def _clear_state(self, indices):
        """Forget these bots' render state, so a stopped job's markers
        don't sit frozen on the plot while other jobs keep drawing."""
        with self._gui_lock:
            for i in indices:
                self._gui_state.pop(i, None)

    def stop_goal_jobs(self, ids=None, timeout=5.0):
        """Stop the goal jobs owning any of `ids` (all of them if ids is
        None) and leave every other job running.

        The per-bot counterpart of stopAll(): re-assigning bots 1 and 2
        must stop whatever those two were doing without disturbing the
        job flying bot 3. Deliberately does NOT call teardown() -- the
        plot and the surviving jobs' state have to outlive this.
        """
        wanted = None if ids is None else set(ids)
        targets = [
            job for job in self.goal_jobs if wanted is None or (job["ids"] & wanted)
        ]
        if not targets:
            return []

        # Signal them all before joining any, so they wind down
        # concurrently instead of each waiting out the previous tick.
        for job in targets:
            job["stop"].set()
        for job in targets:
            # stop() also sets the Thread's own event, or Thread.run()
            # would just re-invoke the worker after it returned.
            job["thread"].stop()
            job["thread"].join(timeout=timeout)
            if job["thread"].is_alive():
                print(
                    f"WARNING: goal job {sorted(job['ids'])} did not stop "
                    f"within {timeout}s"
                )

        dead = {id(job) for job in targets}
        self.goal_jobs = [j for j in self.goal_jobs if id(j) not in dead]
        self._sync_goal_slot()
        stopped = [sorted(job["ids"]) for job in targets]
        print(f"Stopped goal jobs: {stopped}")
        return stopped

    def stopAll(self, call_back_action=None, timeout=5.0):
        """Stop every running function and tear down once they're all gone.

        Signals ALL the workers before joining any of them, so they wind
        down concurrently rather than each waiting out the previous
        one's tick. They share the isToStop flag, which is why no worker
        may clear it itself -- teardown() does that at the end, after
        every one of them has exited.

        Returns the names of the functions that were stopped.
        """
        if call_back_action is None:
            call_back_action = self.stopAction

        running = self.running_funcs()
        if not running:
            print("No running function to stop")
            return []

        # Each goal job's OWN event too: the shared isToStop that
        # call_back_action sets is not the only flag they watch.
        for job in self.goal_jobs:
            job["stop"].set()

        for name, slot in running:
            for thread in self._threads_of(slot):
                print(f"Stopping: {name}")
                thread.stop(call_back_action)

        stopped = []
        for name, slot in running:
            alive = False
            for thread in self._threads_of(slot):
                thread.join(timeout=timeout)
                if thread.is_alive():
                    alive = True
            if alive:
                # Daemon threads, so this can't block interpreter exit,
                # but the state it owns can't be safely cleared either.
                print(f"WARNING: '{name}' did not stop within {timeout}s")
            else:
                stopped.append(name)

        # Must run: it clears isToStop (nothing else does), empties
        # goal_jobs and the current_function slots, and closes the plot.
        # Without it a stop leaves isToStop True and dead jobs in the
        # list, so the NEXT goal skips its `if not self.goal_jobs`
        # reset and its worker breaks on the first tick.
        self.teardown()
        print(f"Stopped all: {stopped}")
        return stopped

    def gui_tick(self):
        """Call repeatedly from the main thread to drive the plot."""
        if not self.Variables.gui: return
        if self.gui is None:
            return
        if self._gui_close_requested:
            self.gui.close()
            self.gui = None
            self._gui_close_requested = False
            return
        with self._gui_lock:
            state = list(self._gui_state.items())
        if not state:
            # Nothing running, but keep pumping: update() ends in
            # plt.pause(), and without it the window stops redrawing
            # and the desktop greys it out as unresponsive. Throttled
            # to ~20Hz -- main.py calls this in a tight spin loop, and
            # a full redraw per iteration pegs a core for nothing.
            now = time()
            if now - self._last_idle_pump >= 0.05:
                self._last_idle_pump = now
                self.gui.update()
            return
        # Rebuild the whole-swarm view that every concurrent job fed into.
        num_bots = len(self.s.swarm)
        goals = []
        drone_positions = [None] * num_bots
        lookahead_targets = [None] * num_bots
        for i, (goal, position, lookahead) in state:
            if i >= num_bots:
                continue
            if goal is not None:
                goals.append(goal)
            drone_positions[i] = position
            lookahead_targets[i] = lookahead
        self.gui.show_goals(goals)
        self.gui.show_gps_positions(drone_positions)
        self.gui.show_lookahead_targets(lookahead_targets)
        self.gui.update()

    def _guide_and_reposition(self, i, b, x, y, theta, rx, ry):
        """One tick of guidance for a single bot: advance it along its
        line, compute the lookahead target for the real vehicle, and
        optionally push that target to the vehicle via send_reposition.

        Always advances the bot -- it's the path planner and must keep
        moving every tick regardless of vehicle proximity -- and pushes
        the target to the vehicle on the reposition_interval_s clock.
        Returns the computed (tx, ty).

        x, y: bot's pose at the START of this tick (captured by the
        caller before calling this, so the lookahead line is anchored
        to where the bot actually was this tick, not post-move).
        """
        dis = b.dist(rx, ry)
        along = (rx - x) * cos(theta) + (ry - y) * sin(theta)
        lead_state = True if along > 0 else False

        # Route already finished -- either frozen in place (ground/
        # copter) or orbiting the last waypoint (fixedwing's
        # b.loitering, since it can't hover). Either way there's no
        # more progress to make along a route, so none of the
        # lead/chase/resync machinery below -- which exists purely to
        # help the bot and the real drone close a gap and keep moving
        # TOGETHER along one -- applies anymore. Confirmed this
        # matters for loitering too, not just frozen: while loitering,
        # the bot's (x,y,theta) keep revolving around the loiter
        # center, so the normal lookahead carrot (projected `along`
        # the bot's current, constantly-rotating heading) sweeps
        # around with it and never settles -- the real drone chases a
        # point that outruns it, the gap blows past resync_sep_m, and
        # snap_resync fires over and over (teleporting the bot off its
        # own loiter circle each time) instead of the vehicle just
        # holding a stable orbit.
        already_finished = b.done

        if not already_finished and dis > self.Variables.resync_sep_m and lead_state:
            self.log.info(
                f"bot={i+1} type={b.vehicle_type} EVENT=snap_resync "
                f"bot_pos=({x:.1f},{y:.1f}) drone_pos=({rx:.1f},{ry:.1f}) "
                f"dist={dis:.1f}m goal={b.goal if b.goal_given else None}"
            )
            b.x, b.y = rx, ry
            return None, None

        if not already_finished:
            # Keep the bot's speed live-tunable from the web config panel
            # WHILE a mission is already running, not just on the next
            # search/goal/split command -- _populate_sim() only re-resolves
            # speed when a NEW mission command repopulates every bot from
            # scratch, so a bot already mid-route never saw a panel edit
            # take effect until that happened. Variables.speed/copter_speed
            # ALWAYS wins for the bot's speed, by request, even for a bot
            # whose real vehicle has its own detected cruise speed -- this
            # only changes the SIMULATED bot's kinematics, never the real
            # vehicle's own flight speed (send_reposition() never sends a
            # groundspeed).
            panel_speed = (
                self.Variables.copter_speed
                if b.vehicle_type != "fixedwing"
                else self.Variables.speed
            )
            # The real drone has moved ahead of the bot along the bot's own
            # heading (lead_state, computed above from `along`) -- it's
            # catching up to or passing the point it's supposed to be
            # chasing, so bump the bot's speed to re-open its lead instead
            # of just letting the drone walk it down every tick this keeps
            # happening.
            if lead_state:
                panel_speed += self.Variables.lead_speed_boost
            elif dis > self.Variables.chase_sep_m:
                # The BOT is the one too far ahead this time (the drone
                # hasn't passed it, but the gap is past chase_sep_m) --
                # both have to keep moving simultaneously, not just the
                # bot racing off on its own, so ramp its speed down toward
                # a floor the closer `dis` gets to the hard resync distance
                # (resync_sep_m), giving the real drone room to close the
                # gap on its own. The floor respects each vehicle's own
                # nature: a copter can genuinely hover, so it's allowed
                # down to a full stop; a fixedwing can never stop or pivot
                # in place, so it keeps at least half cruise speed even at
                # its slowest -- it still closes the gap, just slower,
                # rather than stalling.
                span = max(self.Variables.resync_sep_m - self.Variables.chase_sep_m, 1e-6)
                frac_over = min((dis - self.Variables.chase_sep_m) / span, 1.0)
                floor_frac = 0.5 if b.vehicle_type == "fixedwing" else 0.0
                panel_speed *= 1.0 - frac_over * (1.0 - floor_frac)
            if b.max_speed != panel_speed:
                b.max_speed = panel_speed
                if b.vehicle_type == "fixedwing":
                    b.min_speed = panel_speed

        # Pre-guidance snapshot, to log a discrete EVENT line below if
        # this tick advances the goal queue, starts loitering, or
        # finishes the route -- the state line alone (logged every
        # reposition cycle) shows WHAT a bot was doing at a given
        # instant, these show the exact tick something CHANGED.
        prev_goal = b.goal
        prev_loitering = b.loitering
        prev_done = b.done

        # Hybrid swarm: each bot's own real vehicle configuration, not
        # one shared default for every type. vehicle.py's per-drone
        # radii[i] is WP_LOITER_RAD (metres) for a fixedwing/quadplane,
        # WPNAV_RADIUS (already converted to metres) for a copter/
        # ground bot -- None if that drone's parameters didn't yield a
        # usable value.
        #
        # fixedwing: can't hover at its final waypoint, so give
        # line_waypoint_guidance() a real orbit radius there instead of
        # literally freezing in place -- prefer the real WP_LOITER_RAD,
        # falling back to the bank-angle-derived turn radius, then the
        # Variables default. b.loitering only turns True once the bot
        # has actually reached that last waypoint -- see guided.py.
        #
        # ground/copter: CAN hover, so it keeps freezing in place
        # (loiter_radius stays None) -- but its own real WPNAV_RADIUS
        # becomes its capture_radius (how close counts as "arrived"),
        # instead of the same hardcoded 50.0 for every bot regardless
        # of type.
        radius = self.vehicle.radii[i] if i < len(self.vehicle.radii) else None
        if b.vehicle_type == "fixedwing":
            loiter_radius = radius or b.turn_radius() or self.Variables.vehicle_loiter_radius_min_m
            capture_radius = 50.0
        else:
            loiter_radius = None
            capture_radius = radius or 50.0

        # Hybrid swarm: how much separation to keep from other bots also
        # has to match each vehicle's own scale. 100m is reasonable for
        # a fast fixedwing with a wide turn radius, but applying it to a
        # copter too creates a real deadlock: a copter whose route runs
        # closer than 100m to another bot's (very plausible on a dense
        # coverage grid -- this swarm's own missions use grid_space=100,
        # right at that threshold) can never get within avoid_min_sep of
        # its own goal without violating the separation constraint, so
        # it just sits there forever, re-aiming at a goal it's
        # geometrically forbidden from reaching. Confirmed directly:
        # parallel routes 100-130m apart captured normally (0m closest
        # approach); at 60m spacing the bot got permanently stuck 39m
        # from its goal. A copter (hovers precisely, WPNAV_RADIUS is
        # only ~2m) needs far less real separation than a fixedwing.
        avoid_sep_m = self._avoid_sep_for_type(b.vehicle_type)
        cmd = line_waypoint_guidance(
            b,
            capture_radius=capture_radius,
            freeze_on_arrival=not b.done,
            avoid_bots=True,
            avoid_weight=30.0,
            avoid_min_sep=avoid_sep_m,
            avoid_radius=avoid_sep_m,
            loiter_radius=loiter_radius,
        )
        cmd.exec(b)
        if b.goal != prev_goal:
            self.log.info(
                f"bot={i+1} type={b.vehicle_type} EVENT=goal_advanced "
                f"from={prev_goal} to={b.goal} queue_remaining={len(b.goal_queue)}"
            )
        if b.loitering and not prev_loitering:
            self.log.info(
                f"bot={i+1} type={b.vehicle_type} EVENT=loiter_started "
                f"center={b.loiter_center} radius={loiter_radius}"
            )
        if b.done and not prev_done and not b.goal_queue and not b.loitering:
            self.log.info(
                f"bot={i+1} type={b.vehicle_type} EVENT=route_finished "
                f"final_goal={b.goal}"
            )

        if already_finished:
            # Route's over -- command the real vehicle to hold at the
            # actual finished point, not a carrot projected
            # lookahead_m beyond it. compute_lookahead_target()'s
            # whole point is to keep a vehicle that's still catching up
            # making progress along the route; once there's no more
            # route, that same formula just sends the vehicle sailing
            # past the goal chasing a point that never gets any
            # closer, which is the "drone keeps moving past the final
            # waypoint" symptom -- and the growing bot<->drone distance
            # from that overshoot is also what was spuriously
            # triggering snap_resync above.
            #
            # Frozen (ground/copter, or a fixedwing leg that didn't
            # need a loiter): hold at the bot's own fixed final
            # position. Loitering (fixedwing): hold at the loiter
            # CENTER instead of the bot's current (constantly
            # revolving) position on the circle -- send_reposition
            # below still passes the real loiter_radius, so the real
            # aircraft orbits that fixed center on its own instead of
            # chasing the sim bot's position around the circle.
            lookahead_m = 0.0
            if b.loitering and b.loiter_center is not None:
                tx, ty = b.loiter_center
            else:
                tx, ty = b.x, b.y
        else:
            if b.vehicle_type == "fixedwing":
                lookahead_m = self.Variables.vehicle_loiter_radius_min_m
            else:
                lookahead_m = self.Variables.copter_lookahead_m

            tx, ty = compute_lookahead_target(
                x,
                y,
                theta,
                rx,
                ry,
                lookahead_m,
            )

        lead_lat, lead_lon = cartToGeo(
            self.Variables.origin,
            self.Variables.endDistance,
            [tx, ty],
        )

        now = time()
        last_sent = self._last_reposition_time.get(i, 0)
        if now - last_sent >= self.Variables.reposition_interval_s:
            # Match the REAL vehicle's arrival behaviour to the sim bot's:
            # 0.0 ("arrive, don't loiter") while still tracking a line/
            # lookahead target, same as before for every vehicle type --
            # and once a fixedwing bot has captured its final waypoint
            # and started orbiting (b.loitering), send that same real
            # orbit radius instead, so the aircraft actually holds there
            # instead of the firmware auto-circling at its own
            # WP_LOITER_RAD. Copters/ground bots never loiter -- they
            # hover -- so this stays 0.0 for them regardless.
            real_loiter_radius = loiter_radius if b.loitering and loiter_radius else 0.0
            send_reposition(
                self.vehicle.drones[i],
                lead_lat,
                lead_lon,
                self.Variables.heights[i],
                loiter_radius=real_loiter_radius,
            )
            self._last_reposition_time[i] = now

            frame = self.vehicle.drones[i].location.global_relative_frame
            self.log.info(
                f"bot={i+1} type={b.vehicle_type} "
                f"bot_pos=({b.x:.1f},{b.y:.1f}) goal={b.goal if b.goal_given else None} "
                f"done={b.done} loitering={b.loitering} queue_remaining={len(b.goal_queue)} "
                f"drone_pos=({rx:.1f},{ry:.1f}) dist_bot_drone={dis:.1f}m "
                f"cmd_latlon=({lead_lat:.6f},{lead_lon:.6f}) drone_latlon=({frame.lat:.6f},{frame.lon:.6f}) "
                f"alt={frame.alt:.0f} lookahead_m={lookahead_m:.1f} "
                f"loiter_radius={loiter_radius} capture_radius={capture_radius:.1f} "
                f"sent_loiter_radius={real_loiter_radius}"
            )
            if getattr(self.Variables, "debug_reposition", False):
                if now - self._last_debug_time.get(i, 0) >= 2.0:
                    self._last_debug_time[i] = now
                    print(
                        f"bot {i+1}: cmd ({lead_lat:.6f}, {lead_lon:.6f}) "
                        f"| drone ({frame.lat:.6f}, {frame.lon:.6f}) alt {frame.alt:.0f} "
                        f"| bot->drone {b.dist(rx, ry) * 2:.0f}m "
                        f"| lead {lookahead_m:.0f}m"
                    )

        return tx, ty

    def search(
        self, center_lat, center_lon, effective_num_uavs, grid_space, coverage_area
    ):
        # Removed bots must not be planned for: the grid is split
        # between the ACTIVE bots only. Handing a removed bot a share of
        # the coverage area would leave that strip unflown -- a hole in
        # the search, and not one self-healing can fill, because those
        # points never belonged to a bot that dropped out mid-run.
        active = [
            i
            for i in range(len(self.vehicle.drones))
            if (i + 1) not in self.removed_bots
        ]
        if not active:
            print("search: every bot has been removed -- nothing to fly")
            return
        if len(active) != int(effective_num_uavs):
            print(
                f"search: planning for {len(active)} active bot(s) "
                f"{[i + 1 for i in active]} -- bot(s) "
                f"{sorted(self.removed_bots)} removed"
            )
        effective_num_uavs = len(active)
        # Reject an impossible area now, before the sim is rebuilt and the
        # plot reset -- those would be left half-done by a late failure.
        check_coverage_area(
            effective_num_uavs, float(grid_space), float(coverage_area)
        )

        pos = self.vehicle.get_positions(
            geoToCart=geoToCart,
            origin=self.Variables.origin,
            endDistance=self.Variables.endDistance,
        )
        self._populate_sim(pos, len(self.vehicle.drones))
        # _populate() rebuilds every bot unfrozen; re-freeze the removed
        # ones so they don't read as active on the plot.
        for i in range(len(self.s.swarm)):
            if (i + 1) in self.removed_bots:
                self.s.swarm[i].done = True
        # Clear the shared stop signal before spawning a worker, or a
        # leftover True from a previous stop would break it instantly.
        self.isToStop = False
        # Fresh plot for this run -- teardown() closes the previous
        # one, and a re-run must not draw onto a closed figure.
        self._reset_gui()
        self.gui.show_env()
        self.gui.show_bots()
        curve = BezierCurveMultiple(
            self.Variables.origin,
            float(center_lat),
            float(center_lon),
            effective_num_uavs,
            int(grid_space),
            int(coverage_area),
            self.Variables.run_id,
        )

        curve.GridFormation()
        curve.generate_bezier_curve()
        goal_sequences = wp.load_goals_for_swarm(curve.filePaths)

        # zip against `active`, so sequence 0 goes to the first bot
        # still in the swarm rather than to bot 1 by position.
        for index, seq in zip(active, goal_sequences):
            self._assign(index, seq)
            # Newest route only: replaces this bot's previous route line.
            xs, ys = zip(*seq)
            self.gui.show_route([index], xs, ys)

        self.gui.run()

        def searchSub():
            self.check_in_time = time()
            while True:
                # Break, but do NOT clear isToStop -- it's the shared
                # signal every running function watches. Clearing it
                # here would stop only whichever worker noticed first
                # and leave the rest running. teardown() clears it,
                # once all of them have exited.
                if self.isToStop:
                    print("Stoped search")
                    break
                sleep(self.Variables.tick_interval_s)
                updates = {}
                try:
                    for i, b in enumerate(self.s.swarm):
                        if (i + 1) in self.removed_bots:
                            continue
                        if (
                            b.done
                            and not self._claim_orphan_work(i)
                            and not b.loitering
                        ):
                            continue
                        v_lat, v_lon = (
                            self.vehicle.drones[i].location.global_relative_frame.lat,
                            self.vehicle.drones[i].location.global_relative_frame.lon,
                        )
                        vx, vy = geoToCart(
                            self.Variables.origin,
                            self.Variables.endDistance,
                            [v_lat, v_lon],
                        )
                        # rx, ry = (
                        #     vx / 2,
                        #     vy / 2,
                        # )  # real vehicle's position in the bot's sim frame

                        # Bot advances its own line every tick regardless of vehicle
                        # proximity -- it's the path planner here -- but arrival_gate
                        # stops it handing off to the NEXT queued waypoint until the
                        # real vehicle has also gotten near the current one, so sim
                        # kinematics (much faster than real flight) can't run the
                        # bot far ahead of where the vehicle actually is.
                        x, y, theta = b.get_pose()

                        tx, ty = self._guide_and_reposition(i, b, x, y, theta, vx, vy)
                        updates[i] = (
                            b.goal if b.goal_exists() else None,
                            (vx, vy),
                            (tx, ty),
                        )
                except Exception:
                    # Thread.run() has no handler, so an exception here
                    # would kill this job outright and the only sign
                    # would be drones that quietly stop being commanded.
                    print("ERROR in worker tick:")
                    traceback.print_exc()
                self._publish_state(updates)

        thread = Thread(target=searchSub)
        thread.start()

        self.current_function["search"] = [True, thread]

    def goal(self, guided_circle_direction, guided_circle_radius, goal_latlon, ids):
        goal_latlon = [
            [x, y]
            for x, y in [
                geoToCart(self.Variables.origin, self.Variables.endDistance, geo)
                for geo in goal_latlon
            ]
        ]
        print(goal_latlon)

        ids = [int(i) for i in ids]

        # A new assignment replaces what THESE bots were doing and
        # nothing else: stop only the jobs that own one of them, and
        # leave every other goal job flying. search() drives every bot
        # in the swarm, so it can't coexist with a per-bot goal at all.
        if self.current_function["search"][0]:
            self.stopAll()
        self.stop_goal_jobs(ids)

        dropped = [i for i in ids if i in self.removed_bots]
        if dropped:
            print(f"goal: ignoring removed bot(s) {dropped} -- add_ them back first")
            ids = [i for i in ids if i not in self.removed_bots]
        if not ids:
            print("goal: no active bots in this command")
            return

        owned = [i - 1 for i in ids]  # bot id N is swarm index N-1

        pos = self.vehicle.get_positions(
            geoToCart=geoToCart,
            origin=self.Variables.origin,
            endDistance=self.Variables.endDistance,
        )

        if len(self.s.swarm) != len(self.vehicle.drones):
            # First goal of the session: the swarm doesn't exist yet.
            # Only here, because _populate() DESTROYS and re-creates
            # every bot -- doing that on each command would wipe the
            # goal sequence and pose of the bots other jobs are flying.
            self._populate_sim(pos, len(self.vehicle.drones))

        # Snap just THIS command's bots onto their drone's live x,y (and
        # heading) every time, so a new leg always starts from where the
        # aircraft actually is -- a bot that sat idle, or froze at its
        # last goal, would otherwise set off from a stale sim position.
        # Per-bot, so the bots other jobs are flying are left alone.
        for i in owned:
            if 0 <= i < len(self.s.swarm) and i < len(pos):
                bot = self.s.swarm[i]
                bot.x, bot.y, bot.theta = pos[i]
                print(
                    f"Bot {i+1} placed at drone position " f"({bot.x:.1f}, {bot.y:.1f})"
                )

        if not self.goal_jobs:
            # Nothing else is flying, so the plot can be rebuilt. With a
            # job already running, _reset_gui() would drop its artists.
            # Clear the shared stop signal too, or a leftover True from
            # a previous stop would break the new worker instantly.
            self.isToStop = False
            self._reset_gui()
            self.gui.show_env()
            self.gui.show_bots()

        # Newest route only: replaces the old route lines of these bots.
        xs, ys = zip(*goal_latlon)
        self.gui.show_route([i - 1 for i in ids], xs, ys)

        self.gui.run()

        def goalSub(goal_latlon, ids, stop_event):
            # goal_latlon is the SAME sequence for every id in this one
            # command -- spread the bots sharing it into a small
            # formation ring (see _formation_offsets()) so they're not
            # all told to converge on the literal identical coordinate,
            # which avoid_bots' own avoid_min_sep would otherwise make
            # impossible to actually reach (confirmed cause of multiple
            # copters stuck circling one shared goal, never arriving).
            offsets = self._formation_offsets(
                [i for i in owned if 0 <= i < len(self.s.swarm)]
            )
            for i in owned:
                if not 0 <= i < len(self.s.swarm):
                    continue
                print(f"Goal set to bot: {i+1}")
                dx, dy = offsets[i]
                points = (
                    goal_latlon
                    if (dx, dy) == (0.0, 0.0)
                    else [(gx + dx, gy + dy) for gx, gy in goal_latlon]
                )
                self._assign(i, points)

            while True:
                # This job's OWN event, so stopping the goal for one set
                # of bots leaves the others running. isToStop is still
                # honoured: a full stop has to take everything.
                if stop_event.is_set() or self.isToStop:
                    print(f"Goal for bots {ids} stopped")
                    self._clear_state(owned)
                    break
                sleep(self.Variables.tick_interval_s)
                updates = {}
                try:
                    # Bots some OTHER job is flying: hands off. Snapping
                    # them to their raw drone position below would drag
                    # them back every tick and fight that job's guidance.
                    busy = self._owned_indices()
                    for i, b in enumerate(self.s.swarm):
                        v_lat, v_lon = (
                            self.vehicle.drones[i].location.global_relative_frame.lat,
                            self.vehicle.drones[i].location.global_relative_frame.lon,
                        )
                        vx, vy = geoToCart(
                            self.Variables.origin,
                            self.Variables.endDistance,
                            [v_lat, v_lon],
                        )
                        # rx, ry = (
                        #     vx / 2,
                        #     vy / 2,
                        # )
                        if not (i + 1) in ids:
                            # Not ours. If nothing else is flying it either,
                            # park its marker on the drone's live position so
                            # the plot still tracks that aircraft.
                            if i not in busy:
                                b.x, b.y, b.theta = (
                                    vx,
                                    vy,
                                    compass_to_math_rad(self.vehicle.drones[i].heading),
                                )
                            continue
                        if (i + 1) in self.removed_bots:
                            continue
                        if (
                            b.done
                            and not self._claim_orphan_work(i)
                            and not b.loitering
                        ):
                            if b.done:
                                print(f"Bot: {i+1} is completed goal")
                            continue

                        # Bot advances its own line every tick regardless of vehicle
                        # proximity -- it's the path planner here -- but arrival_gate
                        # stops it handing off to the NEXT queued waypoint until the
                        # real vehicle has also gotten near the current one, so sim
                        # kinematics (much faster than real flight) can't run the
                        # bot far ahead of where the vehicle actually is.
                        x, y, theta = b.get_pose()

                        tx, ty = self._guide_and_reposition(i, b, x, y, theta, vx, vy)
                        updates[i] = (
                            b.goal if b.goal_exists() else None,
                            (vx, vy),
                            (tx, ty),
                        )
                except Exception:
                    # Thread.run() has no handler, so an exception here
                    # would kill this job outright and the only sign
                    # would be drones that quietly stop being commanded.
                    print("ERROR in worker tick:")
                    traceback.print_exc()
                self._publish_state(updates)

        stop_event = threading.Event()
        thread = Thread(target=goalSub, args=(goal_latlon, ids, stop_event))
        thread.start()

        self.goal_jobs.append({"ids": set(ids), "thread": thread, "stop": stop_event})
        self._sync_goal_slot()
        print(
            f"Goal thread started for bots {ids} -- running jobs: "
            f"{[sorted(job['ids']) for job in self.goal_jobs]}"
        )

    def AutosplitMission(
        self, selected_uav_ids, grid_space, coverage_area, center_lat_lon_array
    ):
        print("Auto Split Mission")
        active = [
            i
            for i in range(len(self.vehicle.drones))
            if (i + 1) not in self.removed_bots
        ]
        if not active:
            print("Auto Split Mission: every bot has been removed -- nothing to fly")
            return

        if len(active) != len(selected_uav_ids):
            print(
                f"Auto Split Mission: planning for {len(active)} active bot(s) "
                f"{[i + 1 for i in active]} -- bot(s) "
                f"{sorted(self.removed_bots)} removed"
            )

        pos = self.vehicle.get_positions(
            geoToCart=geoToCart,
            origin=self.Variables.origin,
            endDistance=self.Variables.endDistance,
        )
        self._populate_sim(pos, len(self.vehicle.drones))
        # _populate() rebuilds every bot unfrozen; re-freeze the removed
        # ones so they don't read as active on the plot.
        for i in range(len(self.s.swarm)):
            if (i + 1) in self.removed_bots:
                self.s.swarm[i].done = True

        # Hybrid swarm: each bot's own (vehicle-type-appropriate) cruise
        # speed, keyed by uav id -- lets AutoSplitMission give faster
        # bots a bigger strip instead of splitting the area equally.
        drone_speeds = {
            uav_id: self.s.swarm[uav_id - 1].max_speed
            for uav_id in selected_uav_ids
            if 0 <= uav_id - 1 < len(self.s.swarm)
        }
        split = AutoSplitMission(
            origin=self.Variables.origin,
            center_lat_lons=center_lat_lon_array,
            drone_list=selected_uav_ids,
            grid_spacing=int(grid_space),
            coverage_area=int(coverage_area),
            run_id=self.Variables.run_id,
            drone_speeds=drone_speeds,
        )
        split.GroupSplitting(
            center_lat_lons=center_lat_lon_array,
            num_of_drones=len(selected_uav_ids),
            grid_spacing=int(grid_space),
            coverage_area=int(coverage_area),
        )
        print(split.filePath)
        goal_sequences = wp.load_goals_for_swarm(split.filePath)
        # zip against `active`, so sequence 0 goes to the first bot
        # still in the swarm rather than to bot 1 by position.
        for index, seq in zip(active, goal_sequences):
            self._assign(index, seq)
            # Newest route only: replaces this bot's previous route line.
            xs, ys = zip(*seq)
            self.gui.show_route([index], xs, ys)

        self.gui.run()

        def AutoSplitMissionSub():
            while True:
                # Break, but do NOT clear isToStop -- it's the shared
                # signal every running function watches. Clearing it
                # here would stop only whichever worker noticed first
                # and leave the rest running. teardown() clears it,
                # once all of them have exited.
                if self.isToStop:
                    print("Stopped Auto Mission")
                    break
                sleep(self.Variables.tick_interval_s)
                updates = {}
                try:
                    for i, b in enumerate(self.s.swarm):
                        if (i + 1) in self.removed_bots:
                            continue
                        if (
                            b.done
                            and not self._claim_orphan_work(i)
                            and not b.loitering
                        ):
                            continue
                        v_lat, v_lon = (
                            self.vehicle.drones[i].location.global_relative_frame.lat,
                            self.vehicle.drones[i].location.global_relative_frame.lon,
                        )
                        vx, vy = geoToCart(
                            self.Variables.origin,
                            self.Variables.endDistance,
                            [v_lat, v_lon],
                        )
                        # rx, ry = (
                        #     vx / 2,
                        #     vy / 2,
                        # )  # real vehicle's position in the bot's sim frame

                        # Bot advances its own line every tick regardless of vehicle
                        # proximity -- it's the path planner here -- but arrival_gate
                        # stops it handing off to the NEXT queued waypoint until the
                        # real vehicle has also gotten near the current one, so sim
                        # kinematics (much faster than real flight) can't run the
                        # bot far ahead of where the vehicle actually is.
                        x, y, theta = b.get_pose()

                        tx, ty = self._guide_and_reposition(i, b, x, y, theta, vx, vy)
                        updates[i] = (
                            b.goal if b.goal_exists() else None,
                            (vx, vy),
                            (tx, ty),
                        )
                except Exception:
                    # Thread.run() has no handler, so an exception here
                    # would kill this job outright and the only sign
                    # would be drones that quietly stop being commanded.
                    print("ERROR in worker tick:")
                    traceback.print_exc()
                self._publish_state(updates)

        thread = Thread(target=AutoSplitMissionSub)
        thread.start()

        self.current_function["spilt"] = [True, thread]

    def SpecificSplit(self, center_lat_lon_array, uav_array, grid_space, coverage_area):
        print("Specific Split Mission")
        active = [
            i
            for i in range(len(self.vehicle.drones))
            if (i + 1) not in self.removed_bots
        ]
        if not active:
            print(
                "Specific Split Mission: every bot has been removed -- nothing to fly"
            )
            return

        if len(active) != len(uav_array):
            print(
                f"Specific Split Mission: planning for {len(active)} active bot(s) "
                f"{[i + 1 for i in active]} -- bot(s) "
                f"{sorted(self.removed_bots)} removed"
            )

        pos = self.vehicle.get_positions(
            geoToCart=geoToCart,
            origin=self.Variables.origin,
            endDistance=self.Variables.endDistance,
        )
        self._populate_sim(pos, len(self.vehicle.drones))

        for i in range(len(self.s.swarm)):
            if (i + 1) in self.removed_bots:
                self.s.swarm[i].done = True

        # Hybrid swarm: each bot's own (vehicle-type-appropriate) cruise
        # speed, keyed by uav id -- uav_array is a list of per-area id
        # lists (e.g. [[2,4],[5,7],[1]]), so flatten it to cover every
        # id SpecificSplitMission will look up regardless of which area
        # it's in.
        drone_speeds = {
            uav_id: self.s.swarm[uav_id - 1].max_speed
            for group in uav_array
            for uav_id in group
            if 0 <= uav_id - 1 < len(self.s.swarm)
        }
        split = SpecificSplitMission(
            origin=self.Variables.origin,
            center_lat_lons=center_lat_lon_array,
            drone_array=uav_array,
            grid_spacing=grid_space,
            coverage_area=coverage_area,
            run_id=self.Variables.run_id,
            drone_speeds=drone_speeds,
        )
        split.GroupSplitting(
            center_lat_lons=center_lat_lon_array,
            drone_array=uav_array,
            grid_spacing=grid_space,
            coverage_area=coverage_area,
        )

        goal_sequences = wp.load_goals_for_swarm(split.filePaths)
        # zip against `active`, so sequence 0 goes to the first bot
        # still in the swarm rather than to bot 1 by position.
        for index, seq in zip(active, goal_sequences):
            self._assign(index, seq)
            # Newest route only: replaces this bot's previous route line.
            xs, ys = zip(*seq)
            self.gui.show_route([index], xs, ys)

        self.gui.run()

        def SpecificSplitSub():
            while True:
                if self.isToStop:
                    print("Stopped Auto Mission")
                    break
                sleep(self.Variables.tick_interval_s)
                updates = {}
                try:
                    for i, b in enumerate(self.s.swarm):
                        if (i + 1) in self.removed_bots:
                            continue
                        if (
                            b.done
                            and not self._claim_orphan_work(i)
                            and not b.loitering
                        ):
                            continue
                        v_lat, v_lon = (
                            self.vehicle.drones[i].location.global_relative_frame.lat,
                            self.vehicle.drones[i].location.global_relative_frame.lon,
                        )
                        vx, vy = geoToCart(
                            self.Variables.origin,
                            self.Variables.endDistance,
                            [v_lat, v_lon],
                        )
                        # rx, ry = (
                        #     vx / 2,
                        #     vy / 2,
                        # )  # real vehicle's position in the bot's sim frame

                        # Bot advances its own line every tick regardless of vehicle
                        # proximity -- it's the path planner here -- but arrival_gate
                        # stops it handing off to the NEXT queued waypoint until the
                        # real vehicle has also gotten near the current one, so sim
                        # kinematics (much faster than real flight) can't run the
                        # bot far ahead of where the vehicle actually is.
                        x, y, theta = b.get_pose()

                        tx, ty = self._guide_and_reposition(i, b, x, y, theta, vx, vy)
                        updates[i] = (
                            b.goal if b.goal_exists() else None,
                            (vx, vy),
                            (tx, ty),
                        )
                except Exception:
                    # Thread.run() has no handler, so an exception here
                    # would kill this job outright and the only sign
                    # would be drones that quietly stop being commanded.
                    print("ERROR in worker tick:")
                    traceback.print_exc()
                self._publish_state(updates)

        thread = Thread(target=SpecificSplitSub)
        thread.start()

        self.current_function["spilt"] = [True, thread]
