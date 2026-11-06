import numpy as np
from swarm_tasks.controllers.command import Cmd
from swarm_tasks.controllers import bezier
from swarm_tasks.controllers import potential_field

"""
Guided-waypoint navigation for fixedwing bots.

Behaviour:
	- Flies direct-to the bot's current goal (bot.set_goal(x,y))
	- Once within `capture_radius` of the goal, transitions into a
	  loiter: a clockwise (default) circular orbit around the waypoint,
	  held at `loiter_radius`.

The orbit is flown using a vector-field path-following law (Beard &
McLain, "Small Unmanned Aircraft", ch.10) rather than trying to command
the bot straight at the centre point, which a fixedwing can't track
(it can't stop or tighten its turn past its bank-angle limit).

Bot-bot/obstacle avoidance (opt-in via avoid_bots=True) reuses the same
potential-field repulsion the ground-robot tasks already use
(controllers.potential_field.get_field()) -- see _avoidance_offset()
below for how it's adapted to not violate a fixed-wing's turn-rate limit.
"""

DEFAULT_CAPTURE_RADIUS = 5.0
DEFAULT_K_ORBIT = (
    1.0  # Orbit-law gain: higher = snaps to the circle faster, but more aggressive
)
DEFAULT_AVOID_WEIGHT = 1.0
DEFAULT_AVOID_BIAS_DEG = 25.0  # See _avoidance_offset()'s docstring for why this exists


def _default_avoid_radius(bot):
    """
    How far out a bot starts reacting to a neighbour/obstacle. Scaled to
    several turn radii, not just bot size or a single turn radius -- a
    fixedwing can't dodge sideways, it can only bank into a gradual
    curve, so it needs real room (and real time, at cruise speed) to
    actually separate before a close encounter, not just barely enough
    space to complete one tight turn. Falls back to 60m for bots with no
    bank_angle_deg set (turn_radius() is None).
    """
    turn_r = bot.turn_radius()
    return max(4.0 * bot.size, 3.0 * (turn_r if turn_r is not None else 20.0))


def _avoidance_offset(
    bot,
    avoid_radius,
    avoid_weight,
    max_offset,
    bias_deg=DEFAULT_AVOID_BIAS_DEG,
    min_sep=None,
):
    """
    Bot-bot/obstacle repulsion for fixed-wing guidance.

    Two different repulsion laws, selected by whether `min_sep` is given:

    1. min_sep is None (default): built directly on top of
       controllers.potential_field.get_field() -- the same inverse-square
       (1/r^2) repulsion the ground-robot tasks (dispersion, aggregation,
       foraging, ...) already use. Simple and reuses existing code, but
       1/r^2 decays fast: it only pushes hard once bots are ALREADY quite
       close, so the *minimum separation actually achieved* during a pass
       stays small (a few metres) no matter how large avoid_weight gets --
       raising avoid_weight from 15 to 800 only moved the achieved
       clearance from ~0.8m to ~5m in testing. Fine for "don't literally
       collide", not for "always keep 15m clear".

    2. min_sep is a number (metres): a linear "keep-out zone" push
       instead. For each nearby bot within min_sep, the push magnitude is
       avoid_weight * (min_sep - actual_distance) -- i.e. it's exactly
       zero right at min_sep and grows linearly as the gap closes further,
       the same shape as a spring being compressed. This directly targets
       a *specific* standoff distance rather than relying on how fast an
       inverse-square field happens to decay, so "keep at least 15m away"
       means what it says.

    Either way, why this can't just be handed to move() as a raw velocity
    command: right next to another bot the push can still be large,
    which is fine for a ground bot that can stop, but not for a
    fixed-wing, which can only turn at a rate bounded by its bank angle.
    So this returns a *position offset* (metres), magnitude-clamped to
    max_offset, meant to be added to the tracker's target point (see
    bezier_waypoint_guidance) -- letting the existing pure-pursuit/
    curvature machinery turn that into a heading change that's
    automatically bank-angle-safe, the same as it already is for any
    other target point.

    The raw repulsion is also rotated by a small fixed angle (bias_deg,
    clockwise) before being returned. Two bots on an exact head-on
    course produce exactly opposite repulsion vectors with no rotation --
    a textbook potential-field deadlock, both bots pushing straight back
    at each other and never actually separating. A small consistent
    rotational bias breaks that symmetry: both bots turn the same
    rotational sense relative to their own repulsion vector, which
    (like the "always pass on the right" convention in real aviation/
    maritime rules) makes them curve to pass on each other's right
    instead of meeting the deadlock head-on.

    Returns: np.array([dx, dy]), magnitude <= max_offset.
    """
    if bot.sim is None or avoid_weight <= 0:
        return np.array([0.0, 0.0])

    if min_sep is not None:
        vec = np.zeros(2)
        bx, by = bot.get_position()
        for other in bot.sim.swarm:
            if other is bot:
                continue
            d = bot.dist(other.x, other.y)
            if d >= min_sep or d < 1e-6:
                continue
            penetration = min_sep - d  # 0 right at min_sep, grows as they close further
            direction = np.array([bx - other.x, by - other.y]) / d
            vec += avoid_weight * penetration * direction
    else:
        field_cmd = potential_field.get_field(
            bot.get_position(),
            bot.sim,
            weights={
                "bots": avoid_weight,
                "obstacles": avoid_weight,
                "borders": 0,
                "goal": 0,
                "items": 0,
            },
            max_dist=avoid_radius,
        )
        vec = field_cmd.vec

    mag = np.linalg.norm(vec)
    if mag < 1e-9:
        return np.array([0.0, 0.0])

    theta = -np.radians(
        bias_deg
    )  # Negative = clockwise ("pass right") in standard x-right/y-up axes
    c, s = np.cos(theta), np.sin(theta)
    vec = np.array([c * vec[0] - s * vec[1], s * vec[0] + c * vec[1]])

    mag = np.linalg.norm(vec)
    if mag > max_offset:
        vec = vec * (max_offset / mag)
    return vec


def line_waypoint_guidance(
    bot,
    capture_radius=None,
    lookahead=None,
    freeze_on_arrival=False,
    arrival_gate=None,
    avoid_bots=False,
    avoid_radius=None,
    avoid_weight=DEFAULT_AVOID_WEIGHT,
    avoid_max_offset=None,
    avoid_min_sep=None,
    loiter_radius=None,
    clockwise=True,
    k_orbit=DEFAULT_K_ORBIT,
):
    """
    Flies the STRAIGHT LINE SEGMENT between where this leg started
    (bot.line_start -- wherever the bot was when bot.goal was last set,
    i.e. at/near the previous waypoint) and the current goal (bot.goal),
    tracked with a pure-pursuit lookahead point on that line.

    Unlike waypoint_guidance() (aims the heading directly at bot.goal --
    a straight chord in theory, but nothing corrects it back onto that
    chord if it drifts, e.g. under avoidance) or bezier_waypoint_guidance()
    (plans one open-loop curvature-constrained curve from wherever the bot
    happened to be when the goal changed, using only its heading at that
    instant -- the curve can bow away from the direct line and there's no
    further correction once planned), this recomputes the closest point on
    the ACTUAL line segment from bot.line_start to bot.goal every tick, so
    any lateral drift off that line is corrected back onto it continuously
    -- the bot ends up actually flying ALONG the line between each pair of
    consecutive goals, not just approximately near it.

    The lookahead point (rather than aiming straight at the closest point
    on the line) is what keeps this flyable for a fixed-wing: aiming
    directly at the closest point commands an instant 90-degree correction
    the moment the bot is off the line at all, same problem
    waypoint_guidance has aiming at a point up close. Projecting a
    lookahead distance further along the line blends "get back on line"
    with "keep making progress along it", the same role lookahead plays
    in bezier_waypoint_guidance's curve tracking.

    Args:
            bot: Bot instance. Must have a goal set via bot.set_goal(x,y).
            capture_radius: distance to goal at which the bot is considered
                    arrived. Defaults to DEFAULT_CAPTURE_RADIUS.
            lookahead: distance ahead (along the line) of the bot's closest
                    point on it to steer towards. Defaults to the bot's own turn
                    radius, same reasoning as bezier_waypoint_guidance's lookahead.
            freeze_on_arrival, avoid_bots, avoid_radius, avoid_weight,
                    avoid_max_offset, avoid_min_sep: same as waypoint_guidance()/
                    bezier_waypoint_guidance() -- freeze_on_arrival auto-advances
                    through bot.goal_queue (Bot.set_goal_sequence()) and only
                    freezes once it's exhausted; the avoid_* args bias the
                    lookahead target away from nearby bots/obstacles the same way
                    bezier_waypoint_guidance does.
            arrival_gate: optional callable, arrival_gate(bot) -> bool. Only
                    consulted once bot.dist(bot.goal) < capture_radius. Returning
                    False holds off advancing/freezing this tick, so the bot
                    keeps flying/re-approaching the goal instead. Use this to
                    gate advancement on something the SIM can't see on its own, e.g.
                    "has the REAL vehicle's GPS position (not just bot.x/y) also
                    reached this point" -- bot.x/y can satisfy capture_radius well
                    before a real aircraft physically gets there (sim kinematics
                    run far faster than real flight), so without this the bot
                    would hand off the NEXT goal before the vehicle finished the
                    current leg. None (default) imposes no extra condition.
            loiter_radius: if set (metres) and freeze_on_arrival is True, the
                    bot orbits the LAST waypoint of the sequence (bot.goal_queue
                    exhausted) instead of literally freezing there once it's
                    captured -- unlike a full stop, this is physically realistic
                    for a fixed-wing (it can't hover, but it can hold a steady
                    turn). bot.done still latches True at that point (the route
                    IS finished -- Bot.set_goal()/resume() still un-freeze it the
                    same way, e.g. if self-healing hands it another bot's
                    leftover work), and bot.loitering/bot.loiter_center are set
                    the same way waypoint_guidance()'s loiter does. None/0
                    (default): freeze in place instead, exactly as before.
            clockwise: loiter direction (True=clockwise, as seen from above),
                    only meaningful when loiter_radius is set.
            k_orbit: gain of the orbit-following law, same as
                    waypoint_guidance()'s -- only meaningful when loiter_radius
                    is set.

    Returns: Cmd
    """
    if not bot.goal_exists():
        return Cmd(dir_=bot.theta, speed=bot.max_speed)

    capture_radius, _ = _resolve_radii(bot, capture_radius, loiter_radius)

    if lookahead is None:
        turn_r = bot.turn_radius()
        lookahead = turn_r if turn_r is not None else 15.0

    gx, gy = bot.goal
    d_to_goal = bot.dist(gx, gy)

    if freeze_on_arrival:
        if bot.done:
            if bot.loitering:
                center = (
                    bot.loiter_center if bot.loiter_center is not None else (gx, gy)
                )
                return _orbit_cmd(bot, center, loiter_radius, clockwise, k_orbit)
            return Cmd(dir_=bot.theta, speed=bot.max_speed)
        if d_to_goal < capture_radius:
            if bot.goal_queue:
                # More waypoints queued behind this one: advancing to the
                # next one does NOT freeze the bot (advance_goal() only
                # freezes once the queue is empty -- see its docstring), so
                # it's always safe here regardless of nearby frozen
                # neighbours -- gating this too would deadlock whenever
                # this waypoint sits within min_clearance of another bot's
                # already-frozen one: "captured this goal" (< capture_radius)
                # and "clear of that neighbour" (> min_clearance) become
                # mutually exclusive whenever the two goals are less than
                # capture_radius+min_clearance apart, so the bot could
                # never advance and just orbited near this waypoint
                # forever instead of continuing its route (confirmed via
                # simulation). The FINAL freeze below has the identical
                # deadlock risk and also does not gate on neighbour
                # clearance -- finishing every bot's route matters more
                # than enforcing avoid_min_sep at exactly this one point.
                bot.advance_goal()  # queue non-empty -> always returns False
                gx, gy = bot.goal
                d_to_goal = bot.dist(gx, gy)
            else:
                # arrival_gate is a different, still-honoured mechanism
                # (real-vehicle sync, not inter-bot proximity).
                gated = arrival_gate is None or arrival_gate(bot)
                if gated:
                    bot.advance_goal()  # queue empty -> freezes (bot.done=True)
                    if loiter_radius:
                        # Circle the LAST waypoint instead of literally
                        # freezing there. bot.done stays True (route
                        # finished -- set_goal()/resume() still un-freeze
                        # it); the caller must keep calling this function
                        # (and b.move()) every tick for a done-but-
                        # loitering bot, or the orbit is computed but
                        # never actually flown.
                        bot.loitering = True
                        bot.loiter_center = (gx, gy)
                        return _orbit_cmd(
                            bot, bot.loiter_center, loiter_radius, clockwise, k_orbit
                        )
                    return Cmd(dir_=bot.theta, speed=bot.max_speed)
                # else: arrival_gate said not yet -- keep tracking this
                # same (uncaptured) line target instead of freezing/
                # orbiting before the real vehicle has also arrived.

    # New leg? Anchor its start at wherever the bot is right now -- thanks
    # to capture_radius, that's at/near the PREVIOUS goal, so the segment
    # being tracked is (approximately) exactly the leg between the two
    # consecutive waypoints, as intended.
    if bot.line_start is None or bot.line_start_goal != bot.goal:
        bot.line_start = (bot.x, bot.y)
        bot.line_start_goal = bot.goal

    target = _lookahead_point_on_line(bot, bot.line_start, bot.goal, lookahead)

    if avoid_bots:
        r = avoid_radius if avoid_radius is not None else _default_avoid_radius(bot)
        if avoid_max_offset is not None:
            max_off = avoid_max_offset
        else:
            turn_r = bot.turn_radius()
            max_off = 2.0 * (turn_r if turn_r is not None else 20.0)
        target = target + _avoidance_offset(
            bot, r, avoid_weight, max_off, min_sep=avoid_min_sep
        )

    # Expose the ACTUAL point being steered at (post-avoidance-offset) --
    # not just bot.goal, the raw discrete waypoint -- so a caller driving
    # real hardware (e.g. hardware/ardupilot_link.py's send_target()) can
    # hand the real vehicle this same avoidance-adjusted point instead of
    # the bare goal. Sending only bot.goal means the real aircraft's own
    # flight controller flies straight at the raw waypoint once, with NO
    # benefit from avoid_bots -- that offset only ever reached bot.x/y in
    # the sim. See test_goal_csv_sequence.py's main loop for the send side.
    bot.last_guidance_target = (float(target[0]), float(target[1]))

    # The actual bank-angle-derived turn-rate limit is enforced in
    # Bot.move() (max_turn_speed * step_size, correctly in rad/tick) --
    # there was a redundant clip here too, but it compared bearing
    # (radians, max +-pi =~ 3.14) directly against bot.bank_angle_deg
    # (degrees, e.g. 30), so for any plausible bank angle it could never
    # actually constrain anything, and crashed outright for a ground/
    # copter bot built with bank_angle_deg left at its None default
    # (bank angle isn't a meaningful concept for an omnidirectional
    # vehicle type in the first place).
    bearing = np.arctan2(target[1] - bot.y, target[0] - bot.x)
    return Cmd(dir_=bearing, speed=bot.max_speed)


def _resolve_radii(bot, capture_radius, loiter_radius):
    if capture_radius is None:
        capture_radius = DEFAULT_CAPTURE_RADIUS
    if loiter_radius is None:
        loiter_radius = bot.turn_radius()
        if loiter_radius is None:
            loiter_radius = DEFAULT_CAPTURE_RADIUS
    return capture_radius, loiter_radius


def _lookahead_point(bot, path, lookahead):
    """
    Finds the point on `path` closest to the bot, then walks forward
    along the path (by cumulative arc length) until `lookahead` distance
    has been covered, and returns that point. Clamped to the last point
    of the path if the lookahead runs past the end.
    """
    pos = np.array([bot.x, bot.y])
    dists = np.linalg.norm(path - pos, axis=1)
    closest_idx = int(np.argmin(dists))

    acc = 0.0
    for i in range(closest_idx, len(path) - 1):
        seg = np.linalg.norm(path[i + 1] - path[i])
        if acc + seg >= lookahead:
            # Interpolate within this segment
            remaining = lookahead - acc
            frac = remaining / seg if seg > 1e-9 else 0.0
            return path[i] + frac * (path[i + 1] - path[i])
        acc += seg

    return path[-1]


def _lookahead_point_on_line(bot, start, end, lookahead):
    """
    Point on the line SEGMENT from `start` to `end`, `lookahead` metres
    ahead of the bot's closest point on that (infinite) line -- clamped
    to `end` if that would overshoot the segment, and to `start` if the
    bot is behind it. Straight-line analogue of _lookahead_point(), which
    does the same walk-forward-by-arc-length trick over a sampled Bezier
    path instead of an analytic line.
    """
    sx, sy = start
    ex, ey = end
    line_vec = np.array([ex - sx, ey - sy])
    line_len = np.linalg.norm(line_vec)
    if line_len < 1e-6:
        return np.array([ex, ey])
    line_dir = line_vec / line_len

    rel = np.array([bot.x - sx, bot.y - sy])
    proj = np.dot(rel, line_dir)  # Distance along the line to the bot's closest point
    target_proj = np.clip(proj + lookahead, 0.0, line_len)
    return np.array([sx, sy]) + target_proj * line_dir


def _orbit_cmd(bot, center, radius, clockwise=True, k_orbit=DEFAULT_K_ORBIT):
    """
    Vector-field orbit-following guidance law.
    Converges onto a circle of `radius` around `center`, travelling
    clockwise (as seen from above, standard x-right/y-up convention)
    when clockwise=True.
    """
    cx, cy = center
    dx = bot.x - cx
    dy = bot.y - cy
    d = np.hypot(dx, dy) + 1e-6  # Distance from centre; avoid /0

    chi_o = np.arctan2(dy, dx)  # Bearing from centre to bot

    lam = -1.0 if clockwise else 1.0
    chi_d = chi_o + lam * (np.pi / 2 + np.arctan(k_orbit * (d - radius) / radius))

    return Cmd(dir_=chi_d, speed=bot.max_speed)
