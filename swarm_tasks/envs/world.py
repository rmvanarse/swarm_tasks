import sys, yaml, os
import numpy as np

import shapely
from shapely.geometry import Polygon

"""
Each env is stored as a yaml file listing size, resolution & obstacles.
Loader will load the shapely polygon array in a World object.
"""

#Relative paths (May cause complications while running from different folders)
envs_path = os.path.dirname(os.path.abspath(__file__))
worlds_path = os.path.join(envs_path, 'worlds')

DEFAULT_SIZE = (20.0, 20.0)		#Original hardcoded default, kept as fallback only
DEFAULT_RESOLUTION = 1.0		#metres per grid cell, used by anything doing per-cell bookkeeping


class World:
	"""
	Defines the operational area for a simulation/mission: its extent
	(size, in metres) and the resolution used for any grid-based
	bookkeeping (coverage tracking, etc -- see envs/coverage_grid.py).

	Nothing in this class allocates memory proportional to
	size/resolution -- constructing a World(width=10000, height=10000)
	costs exactly the same as World(width=20, height=20). Anything that
	*does* need per-cell storage decides for itself (based on
	self.grid_rows*self.grid_cols) whether to actually allocate a dense
	array or fall back to a sparse structure -- see CoverageGrid.

	Configurable three ways (in priority order):
		1. size=(width, height) tuple, in metres
			[original interface, kept for backward compatibility]
		2. width=..., height=...
		3. filename=... (a *.yaml under envs/worlds/ with
			'size: {x, y}' and an optional 'resolution' key)

	Any of these support arbitrary scales -- 20x20 m for a quick test,
	10km x 10km for a real mission, or larger -- with no code changes:
	only the numbers passed in change.
	"""
	def __init__(self, size=None, width=None, height=None, \
			resolution=None, obstacles=[], filename=None):

		if filename is not None:
			file = os.path.join(worlds_path, filename)
			self.size, self.resolution, self.obstacles = self.load_yaml(file)
		else:
			if size is not None:
				self.size = (float(size[0]), float(size[1]))
			elif width is not None and height is not None:
				self.size = (float(width), float(height))
			else:
				self.size = DEFAULT_SIZE
			self.resolution = float(resolution) if resolution is not None else DEFAULT_RESOLUTION
			self.obstacles = obstacles

		if self.size[0] <= 0 or self.size[1] <= 0:
			raise ValueError("World size must be positive: got %s" % (self.size,))
		if self.resolution <= 0:
			raise ValueError("World resolution must be positive: got %s" % self.resolution)

		#Grid dimensions implied by size/resolution. These are just two
		#integers -- computing them is free regardless of world size, so
		#any consumer (CoverageGrid, a grid-search task, ...) can check
		#n_cells up front and choose a dense vs. sparse strategy before
		#allocating anything.
		self.grid_cols = int(np.ceil(self.size[0]/self.resolution))
		self.grid_rows = int(np.ceil(self.size[1]/self.resolution))
		self.n_cells = self.grid_cols*self.grid_rows

	def load_yaml(self, filename):
		f = open(filename)
		dict_ = yaml.safe_load(f)
		obstacles = []
		size = (float(dict_['size']['x']), float(dict_['size']['y']))
		resolution = float(dict_['resolution']) if dict_.get('resolution') else DEFAULT_RESOLUTION

		if dict_['obstacles'] != None:
			obstacle_list = dict_['obstacles']
			for o in obstacle_list:
				obstacles.append(Polygon(o))

		name = dict_['name']

		print("Loaded world: "+name+"  size="+str(size)+"  resolution="+str(resolution))
		return size, resolution, obstacles

	def to_cell(self, x, y):
		"""World (x,y) metres -> (row, col) grid index at this world's resolution."""
		col = int(np.clip(x/self.resolution, 0, self.grid_cols-1))
		row = int(np.clip(y/self.resolution, 0, self.grid_rows-1))
		return row, col

	def to_world(self, row, col):
		"""(row, col) grid index -> world (x,y) metres, at the cell centre."""
		x = (col+0.5)*self.resolution
		y = (row+0.5)*self.resolution
		return x, y

	def contains(self, x, y):
		return 0 <= x <= self.size[0] and 0 <= y <= self.size[1]
