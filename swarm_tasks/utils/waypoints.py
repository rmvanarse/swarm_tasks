import csv

"""
CSV waypoint loading -- one goal-sequence file per bot.

Each bot gets its own CSV of (x, y) points, in world units (metres),
one waypoint per row, visited in file order:

	x,y
	60,60
	250,400
	450,100

A header row is optional and auto-detected (if the first row's cells
aren't both numbers, it's skipped). Extra columns (z/altitude/label/...)
are ignored, so richer CSVs work fine too.

Pair this with Bot.set_goal_sequence() (utils/robot.py) and
controllers.guided's freeze_on_arrival=True: each bot then flies its own
list of goals in order, automatically advancing to the next one every
time it captures the current one, and freezes once it's worked through
the whole list -- no per-bot logic needed in the calling script beyond
loading the CSV and calling set_goal_sequence() once at the start.
"""


def load_goals_csv(filepath):
	"""
	Loads an ordered list of (x, y) waypoints from a single CSV file.

	Returns: list of (float, float) tuples, in file order.
	Raises: ValueError if the file has no usable numeric (x, y) rows.
	"""
	with open(filepath, newline='') as f:
		rows = list(csv.reader(f))

	if not rows:
		raise ValueError(f"{filepath}: no rows found")

	#Auto-detect a header row: if the first row's first two cells aren't
	#both numbers, treat it as a header and skip it.
	start = 0
	try:
		float(rows[0][0])
		float(rows[0][1])
	except (ValueError, IndexError):
		start = 1

	waypoints = []
	for i, row in enumerate(rows[start:], start=start):
		if not row or len(row) < 2:
			continue		#Skip blank/short lines rather than failing the whole file
		try:
			x, y = float(row[0]), float(row[1])
		except ValueError:
			raise ValueError(f"{filepath}: row {i} is not numeric x,y: {row}")
		waypoints.append((x, y))

	if not waypoints:
		raise ValueError(f"{filepath}: no waypoint rows found")

	return waypoints


def load_goals_for_swarm(filepaths):
	"""
	Loads one CSV per bot. filepaths is a list of paths, one per bot, in
	swarm order (e.g. matching zip(sim.swarm, filepaths)).

	Returns: list of waypoint lists, same order as filepaths -- pass
	each one to the matching bot's set_goal_sequence().
	"""
	return [load_goals_csv(fp) for fp in filepaths]
