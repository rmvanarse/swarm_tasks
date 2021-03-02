import numpy as np
import shapely #may need submodules

class Item:
	def __init__(self, polygon):
		self.polygon = polygon #May need only dimensions
		self.type = None

	def get_position(self):
		"""
		Returns the centroid of self.polygon
		"""
		p = self.polygon.centroid
		return p.x, p.y

class Attractor(Item):
	def __init__(self, polygon):
		super().__init__(polygon)
		self.type = 'attractor'
	

class Clutter(Item):
	def __init__(self, polygon, weight):
		super().__init__(polygon)
		self.type = 'clutter'
		self.weight = weight

class Obstacle(Item):
	#Enhancement
	pass
