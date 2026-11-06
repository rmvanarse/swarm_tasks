import numpy as np

"""
On-demand lawnmower (boustrophedon) waypoint generation for area-coverage
missions over large worlds.

A naive implementation builds the full list of turn points for the whole
world up front. That's harmless for a small test world, but for a fine
swath spacing over a multi-kilometre world it's thousands of points
computed (and, if fed through generate_multi_waypoint_path(), Bezier-planned)
before the mission has even started -- and all of it wasted if the
mission is re-planned or aborted after the first leg.

lawnmower_waypoints() is a generator: each waypoint is computed only when
asked for, relative to the world's configured size, so cost stays O(1)
per waypoint regardless of how large the operational area is or how fine
the swath spacing is. Pair it with
controllers.bezier.generate_multi_waypoint_path_stream() to also plan
each Bezier leg lazily, so an entire multi-kilometre mission is never
held in memory at once -- only the leg currently being flown.
"""


def lawnmower_waypoints(world, swath, origin=(0.0, 0.0), margin=0.0):
	"""
	Yields (x, y) turn points for a boustrophedon sweep covering `world`,
	spaced `swath` world-units (e.g. metres) apart, one leg at a time.

	Args:
		world: a swarm_tasks.envs.world.World (or anything exposing a
			.size = (width, height) tuple in the same units as swath) --
			the sweep automatically covers exactly this area, at
			whatever scale it's configured for, with no changes needed
			here.
		swath: lateral spacing between passes (e.g. sensor/coverage
			width).
		origin: (x, y) of the sweep's starting corner.
		margin: inset from the world edges on all sides.

	Yields: (x, y) tuples, alternating the long-axis (world height)
		direction each pass. Consume one at a time (e.g.
		`bot.set_goal(*wp)` then wait for `bot.reached_goal()`) -- the
		full route is never materialized as a list, so this scales to
		any world size or swath fineness without a large up-front
		allocation.
	"""
	if swath <= 0:
		raise ValueError("swath must be > 0")

	ox, oy = origin
	width, height = world.size
	x = ox + margin
	y_min, y_max = oy + margin, oy + height - margin
	x_max = ox + width - margin

	going_up = True
	while x <= x_max + 1e-9:
		if going_up:
			yield (x, y_min)
			yield (x, y_max)
		else:
			yield (x, y_max)
			yield (x, y_min)
		going_up = not going_up
		x += swath


def num_passes(world, swath, margin=0.0):
	"""Cheap O(1) estimate of how many lawnmower passes lawnmower_waypoints()
	will yield for this world/swath -- useful for progress reporting
	without having to drain the generator."""
	width, _ = world.size
	usable_width = max(0.0, width - 2*margin)
	return int(np.floor(usable_width/swath)) + 1
