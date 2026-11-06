import numpy as np

"""
Scalable coverage tracking for area-coverage tasks.

The original implementation allocated `np.zeros(world.size, dtype=bool)`
directly off the world's dimensions -- fine at a 20x20m test-world scale
(400 cells), but it doesn't scale: a 10km x 10km world at 1m resolution
is 1e8 cells (~100MB as bool, still just about workable); a 50km x 50km
world at 1m is 2.5e9 cells (~2.5GB) purely to say "nothing's been
visited yet".

CoverageGrid fixes this by choosing its own backing storage based on the
*cell count implied by the world's configured size and resolution*,
transparently to callers:

  - dense  (n_cells <= DENSE_CELL_LIMIT): a numpy bool array, exactly
    like the original -- fast, simple, fine for test/small-mission scale.
  - sparse (n_cells >  DENSE_CELL_LIMIT): a python set of visited
    (row, col) cells. Memory then scales with *area actually flown over
    / coverage radius*, not with the world's total size -- sweeping a
    few km of track across a 50km x 50km world costs a few thousand set
    entries, not billions of pre-allocated cells.

Both backends share the same x,y (metres) <-> (row, col) conversion via
world.resolution, so a caller never needs to know or branch on which
mode is active for normal use (mark / fraction_covered / is_covered).
"""

DENSE_CELL_LIMIT = 20_000_000		#~20MB as bool; above this, switch to sparse


class CoverageGrid:
	def __init__(self, world):
		self.world = world
		self.resolution = world.resolution
		self.rows = world.grid_rows
		self.cols = world.grid_cols
		self.n_cells = world.n_cells

		self.sparse = self.n_cells > DENSE_CELL_LIMIT
		if self.sparse:
			self._visited = set()
			print("CoverageGrid: %d cells (%.0fx%.0fm @ %.2fm/cell) -- "
				"using sparse (visited-set) backend" % \
				(self.n_cells, world.size[0], world.size[1], self.resolution))
		else:
			self._dense = np.zeros((self.rows, self.cols), dtype=bool)

	def to_cell(self, x, y):
		return self.world.to_cell(x, y)

	def mark(self, x, y, r=1.0):
		"""Marks every cell within `r` world-units (e.g. metres) of (x,y) as covered."""
		r_cells = max(1, int(round(r/self.resolution)))
		row, col = self.to_cell(x, y)
		r0, r1 = max(0, row-r_cells), min(self.rows, row+r_cells+1)
		c0, c1 = max(0, col-r_cells), min(self.cols, col+r_cells+1)

		if self.sparse:
			for rr in range(r0, r1):
				for cc in range(c0, c1):
					self._visited.add((rr, cc))
		else:
			self._dense[r0:r1, c0:c1] = True

	def mark_bots(self, bots, r=1.0):
		"""Marks the footprint of every bot's current position -- one CoverageGrid.mark() per bot."""
		for bot in bots:
			self.mark(bot.x, bot.y, r)

	def is_covered(self, x, y):
		row, col = self.to_cell(x, y)
		if self.sparse:
			return (row, col) in self._visited
		return bool(self._dense[row, col])

	def fraction_covered(self):
		if self.n_cells == 0:
			return 0.0
		covered = len(self._visited) if self.sparse else int(np.sum(self._dense))
		return covered/self.n_cells

	def covered_cells(self):
		"""
		Returns an iterable of (row, col) covered cells. Cheap in both
		modes: in sparse mode this is just the visited set (bounded by
		distance flown, not world size); in dense mode it's the nonzero
		indices of the array.
		"""
		if self.sparse:
			return self._visited
		rows, cols = np.nonzero(self._dense)
		return zip(rows.tolist(), cols.tolist())

	def as_dense(self):
		"""
		Materializes a dense bool array of shape (rows, cols).
		Refuses to do so in sparse mode (that's exactly the allocation
		sparse mode exists to avoid) -- use covered_cells() or
		fraction_covered() instead for large worlds.
		"""
		if self.sparse:
			raise MemoryError(
				"CoverageGrid is running in sparse mode (%d cells) -- "
				"refusing to materialize a dense array. Use "
				"covered_cells() or fraction_covered() instead." % self.n_cells)
		return self._dense
