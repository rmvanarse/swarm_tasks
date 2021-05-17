import numpy as np
import shapely #may need submodules
from shapely.geometry import Point, Polygon


class Item:
	"""
	Parent class for all items
	"""
	def __init__(self, polygon):
		self.polygon = polygon #May need only dimensions
		self.type = None
		self.subtype = None
		self.pos = self.get_position()

	def get_position(self):
		"""
		Returns the centroid of self.polygon
		"""
		p = self.polygon.centroid
		return p.x, p.y

class Attractor(Item):
	"""
	Item type used for surround/perimeter behaviour
	"""
	def __init__(self, polygon):
		super().__init__(polygon)
		self.type = 'attractor'
	

class Clutter(Item):
	"""
	Movable items
	"""
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
		*** NOT IMPLEMENTED ***
		*** MAY CONFLICT WITH move() FROM OTHER CHILDREN CLASSES ***

		Translation: 2d vector 
		Rotation: angle
		"""
		#Use poly = shapely.affinity.<rot/translate>(poly)
		pass


class Nest(Attractor):
	"""
	Area for storing gathered resources in foraging
	"""
	def __init__(self, position, radius):
		"""
		Args:
			position: tuple (x,y)
			radius
		"""
		super().__init__(Point(*position).buffer(radius))
		self.subtype = 'nest'
		self.pos = position
		self.radius = radius


class Contamination(Attractor):
	"""
	Used in contamination removal scenarios
	Contamination begins at a hotspot and grows in all directions
	Robots at the perimeter can slow down/reduce the spread
	"""
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
	"""
	Movable circular items
	Used in resource gathering & foraging
	Weight of the item depends on its area
	Difficulty of moving the item depends on its weight
	"""
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

	

	def move(self, dir_, speed):
		#Similar to bot.move
		#Can be used with cmd exec
		speed/=40	#Tuned
		x_,y_ = np.array(self.pos)
		self.move_to((x_+ speed*np.cos(dir_), y_+ speed*np.sin(dir_)))
		return None


	def deplete(self, decrement=1, rate = 0.1):
		#'Consume' the item 
		#Not in use yet
		#Similar to contam update() method
		self.radius -= decrement*rate/(self.radius**2)
		self.polygon = self.pos.buffer(self.radius)


class Obstacle(Item):
	#Enhancement
	pass
