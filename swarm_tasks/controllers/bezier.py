import numpy as np

"""
Curvature-constrained Bezier paths for fixedwing waypoint navigation.

A straight-line-to-waypoint command forces the bot to turn as sharply as
its max_turn_speed allows the instant its heading is off -- fine, but it
produces a hard kink rather than a flown curve. This module instead
generates a quadratic Bezier curve from the bot's current position/heading
to the goal, sized so its curvature never exceeds what the bot can
physically achieve (1/turn_radius from its bank angle + speed).

The bot is then flown along the curve with a pure-pursuit / lookahead
tracker (see controllers/guided.py: bezier_waypoint_guidance).
"""


def quadratic_bezier(P0, P1, P2, t):
	P0, P1, P2 = np.asarray(P0), np.asarray(P1), np.asarray(P2)
	return (1-t)**2*P0 + 2*(1-t)*t*P1 + t**2*P2


def sample_quadratic_bezier(P0, P1, P2, num_points=60):
	ts = np.linspace(0, 1, num_points)
	return np.array([quadratic_bezier(P0, P1, P2, t) for t in ts])


def path_curvatures(points):
	"""
	Discrete (Menger) curvature at each interior point of a polyline.
	kappa = 4*Area(triangle) / (a*b*c)   for the triangle formed by
	three consecutive points; equals 1/R for points lying on a circle
	of radius R.
	Returns: array of length len(points)-2
	"""
	k = np.zeros(max(len(points)-2, 0))
	for i in range(1, len(points)-1):
		p0, p1, p2 = points[i-1], points[i], points[i+1]
		a = np.linalg.norm(p1-p0)
		b = np.linalg.norm(p2-p1)
		c = np.linalg.norm(p2-p0)
		area = 0.5*abs((p1[0]-p0[0])*(p2[1]-p0[1]) - (p2[0]-p0[0])*(p1[1]-p0[1]))
		denom = a*b*c
		k[i-1] = 0.0 if denom < 1e-9 else 4*area/denom
	return k


def generate_curvature_constrained_path(start_pos, start_heading, goal_pos, \
			min_turn_radius, num_points=60, safety_factor=1.15, \
			num_candidates=10):
	"""
	Builds a quadratic Bezier curve P0->P1->P2 where:
		P0 = start_pos
		P2 = goal_pos
		P1 = P0 + d * (cos(start_heading), sin(start_heading))
	d is chosen (via a small grid search) to keep the curve's maximum
	curvature under 1/(min_turn_radius*safety_factor) -- i.e. within
	what the bot can actually fly, with a bit of margin.

	If min_turn_radius is None (e.g. a ground bot with no bank_angle_deg
	set), no constraint is applied and a modest default handle length is
	used, since the sharp-turn clamp is only meaningful for fixedwing bots.

	Returns: Nx2 numpy array of path points from start to goal.
	"""
	P0 = np.asarray(start_pos, dtype=float)
	P2 = np.asarray(goal_pos, dtype=float)
	dir0 = np.array([np.cos(start_heading), np.sin(start_heading)])
	chord = np.linalg.norm(P2-P0)

	if chord < 1e-6:
		return np.array([P0, P2])

	if min_turn_radius is None:
		d = 0.3*chord
		P1 = P0 + d*dir0
		return sample_quadratic_bezier(P0, P1, P2, num_points)

	max_kappa_allowed = 1.0/(min_turn_radius*safety_factor)

	#Grid-search handle length d (as a fraction of chord length);
	#pick the smallest-max-curvature candidate that satisfies the
	#constraint, falling back to the overall best if none do.
	fractions = np.linspace(0.05, 1.5, num_candidates)
	best_pts, best_kappa = None, np.inf
	for frac in fractions:
		d = frac*chord
		P1 = P0 + d*dir0
		pts = sample_quadratic_bezier(P0, P1, P2, num_points)
		kappas = path_curvatures(pts)
		max_k = kappas.max() if len(kappas) else 0.0

		if max_k <= max_kappa_allowed:
			return pts		#First feasible candidate (smallest handle, tightest fit)

		if max_k < best_kappa:
			best_kappa, best_pts = max_k, pts

	#No candidate strictly satisfied the constraint (e.g. very sharp
	#required turn): return the gentlest one found. The bot's own
	#max_turn_speed clamp in move() still physically caps the actual
	#turn, so tracking will just lag the ideal curve slightly here.
	return best_pts


def generate_multi_waypoint_path(start_pos, start_heading, waypoints, \
			min_turn_radius, num_points_per_segment=40, safety_factor=1.15):
	"""
	Chains a curvature-constrained quadratic Bezier segment (see
	generate_curvature_constrained_path) through each waypoint in turn.
	Each segment's start heading is taken from the *end tangent* of the
	previous segment, so consecutive segments join without a heading
	kink -- this is the actual Bezier equivalent of the pursuit-style
	predict_path_with_waypoints() stepwise simulation.

	Args:
		start_pos: (x, y) starting position
		start_heading: starting heading, radians
		waypoints: list of (x, y) points to pass through in order
		min_turn_radius: vehicle's minimum turn radius (m), e.g.
			bot.turn_radius(), or speed/turn_rate if you only have a
			turn rate (rad/s) rather than a bank angle.
		num_points_per_segment: samples per Bezier segment.
		safety_factor: passed to generate_curvature_constrained_path.

	Returns: Nx2 numpy array covering the whole multi-waypoint path
		(joint points are not duplicated between segments).
	"""
	pos = np.asarray(start_pos, dtype=float)
	heading = start_heading
	all_points = []

	for wp in waypoints:
		seg = generate_curvature_constrained_path(
			pos, heading, wp, min_turn_radius,
			num_points=num_points_per_segment, safety_factor=safety_factor)

		all_points.append(seg if len(all_points) == 0 else seg[1:])	#drop duplicate joint point

		#Carry the curve's actual end tangent into the next segment's
		#start heading, so there's no kink at the waypoint
		if len(seg) >= 2:
			heading = np.arctan2(seg[-1][1]-seg[-2][1], seg[-1][0]-seg[-2][0])
		pos = np.asarray(wp, dtype=float)

	return np.concatenate(all_points, axis=0)


def generate_multi_waypoint_path_stream(start_pos, start_heading, waypoints, \
			min_turn_radius, num_points_per_segment=40, safety_factor=1.15):
	"""
	Generator version of generate_multi_waypoint_path(): yields one
	curvature-constrained Bezier *segment* per waypoint, computed lazily,
	instead of building and concatenating the whole route up front.

	Why this matters at scale: generate_multi_waypoint_path() holds
	num_points_per_segment * len(waypoints) points in memory before the
	bot has even started moving. That's negligible for a handful of
	waypoints, but a coverage mission over a multi-kilometre world (see
	modules/coverage_pattern.py) can easily have thousands of legs -- so
	planning the entire route up front means real, avoidable latency and
	memory that scales with the *world*, not with what the aircraft
	actually needs next.

	A caller (e.g. a guidance loop) only ever needs the segment currently
	being flown, so it can pull one segment at a time from this
	generator, hand it to the tracker, and let the previous one be
	garbage-collected -- memory then stays O(one leg) regardless of
	mission length.

	Args:
		start_pos, start_heading: as in generate_multi_waypoint_path().
		waypoints: an iterable (list, or itself a generator, e.g.
			modules.coverage_pattern.lawnmower_waypoints()) of (x, y)
			points to pass through in order.
		min_turn_radius, num_points_per_segment, safety_factor: as in
			generate_multi_waypoint_path().

	Yields: (segment_index, waypoint, Nx2 numpy array of points for that
		segment, end_heading) tuples, one per waypoint, in order.
	"""
	pos = np.asarray(start_pos, dtype=float)
	heading = start_heading

	for i, wp in enumerate(waypoints):
		seg = generate_curvature_constrained_path(
			pos, heading, wp, min_turn_radius,
			num_points=num_points_per_segment, safety_factor=safety_factor)

		if len(seg) >= 2:
			heading = np.arctan2(seg[-1][1]-seg[-2][1], seg[-1][0]-seg[-2][0])
		pos = np.asarray(wp, dtype=float)

		yield i, wp, seg, heading


def predict_path_with_waypoints(initial_pos, initial_heading, speed, turn_rate, \
			waypoints, dt=0.1, max_iter=5000):
	"""
	Drop-in Bezier replacement for a pursuit-style stepwise predictor
	with this same signature. Unlike that version, this pre-plans an
	actual parametric curve through all waypoints (via
	generate_multi_waypoint_path) rather than simulating turn-by-turn.

	dt and max_iter are accepted for interface compatibility but only
	used to size the output resolution -- there's no iterative stepping
	here, so neither can "run away" the way a while-loop predictor can.

	turn_rate (rad/s) + speed (m/s) are converted to the equivalent
	minimum turn radius: R = speed / turn_rate.

	Returns: Nx2 numpy array of (x, y) positions -- same shape/type as
		the stepwise version.
	"""
	if turn_rate <= 0:
		raise ValueError("turn_rate must be > 0")
	min_turn_radius = speed/turn_rate

	#Rough point density matching the original's per-step resolution:
	#total path length isn't known up front, so size each segment by its
	#chord length / (speed*dt), with a sane floor/ceiling.
	pos = np.asarray(initial_pos, dtype=float)
	num_points_per_segment = []
	p = pos
	for wp in waypoints:
		chord = np.hypot(wp[0]-p[0], wp[1]-p[1])
		n = int(np.clip(chord/(speed*dt), 10, max_iter))
		num_points_per_segment.append(n)
		p = np.asarray(wp, dtype=float)

	#generate_multi_waypoint_path uses one point count for all segments;
	#use the max so no segment is under-resolved
	n_points = max(num_points_per_segment) if num_points_per_segment else 40

	return generate_multi_waypoint_path(initial_pos, initial_heading, \
		waypoints, min_turn_radius, num_points_per_segment=n_points)
