import numpy as np
import shapely #may need submodules
from shapely.geometry import Point, Polygon


class Item:
	def __init__(self, polygon):
		self.polygon = polygon #May need only dimensions
		self.type = None
		self.subtype = None

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

		self.min_bots_needed = weight/4
		"""
		Montion is possible when num bots > min_bots_needed
		Max speed is achieved when num bots >= weight
		"""

	def move(self, translation, rotation):
		"""
		Translation: 2d vector 
		Rotation: angle
		"""
		#Use poly = shapely.affinity.<rot/translate>(poly)
		pass

class Nest(Attractor):
	def __init__(self, position, radius):
		"""
		Args:
			position: tuple (x,y)
			radius
		"""
		super().__init__(Point(*postion).buffer(radius))
		self.subtype = 'nest'



class Contamination(Attractor):
	def __init__(self, position, radius):
		"""
		Args:
			position: tuple (x,y)
			radius
		"""
		super().__init__(Point(*position).buffer(radius))
		self.radius = radius
		self.pos = Point(*position)
		self.subtype = 'contamination'

	def update(self, increment=1, rate = 0.005):
		"""
		Increment can be positive or negative
		Radius change is proportional to area and incerment
		"""
		self.radius += increment*rate/(self.radius**2)
		self.polygon = self.pos.buffer(self.radius)
		#print(self.radius)



class Resource(Attractor):
	def __init__(self, position, radius=0.5, density=1):
		"""
		Movable resource, for forraging/gathering
		Args:
			position: (x,y) tuple
			radius
		"""
		super().__init__(Point(*position).buffer(radius))
		self.radius = radius
		self.pos = position
		self.subtype = 'resource'
		self.weight = density*radius*radius
		self.min_bots_needed = self.weight/4

	def move_to(self, new_pos):
		#new_pos: (x,y) tuple
		self.pos = Point(*new_pos)
		self.polygon = self.pos.buffer(self.radius)

	def deplete(self, decrement=1, rate = 0.1):
		#Not in use yet
		#Similar to contam update() method
		self.radius -= decrement*rate/(self.radius**2)
		self.polygon = self.pos.buffer(self.radius)


class Obstacle(Item):
	#Enhancement
	pass
