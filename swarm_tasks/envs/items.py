import sys, yaml, os

import shapely
from shapely.geometry import Polygon

"""
How are items stored and loaded??
"""

envs_path = os.path.dirname(os.path.abspath(__file__))
items_path = os.path.join(envs_path, 'items')


class Contents:
	"""
	
	"""
	def __init__(self, items=[], filename=None):
		if filename!=None:
			file = os.path.join(items_path, filename)
			items = self.load_items(file)
		self.items = items

	def load_items(self, filename):
		"""
		Possible implementations:
		1. [N] Each item is a polygon ==> Other implementations similar to obstacles
		2. [Y] Each item is an object ==> Properties like item.identity

		"""

		items = []

		return items