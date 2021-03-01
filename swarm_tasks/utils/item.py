import numpy as np
import shapely #may need submodules

class Item:
	def __init__(self, polygon):
		self.polygon = polygon #May need only dimensions

class Attractor(Item):
	pass

class Clutter(Item):
	pass

class Obstacle(Item):
	#Enhancement
	pass
