import sys, yaml
import swarm_tasks.envs.world

import shapely
"""
Each env is stored as a yaml file listing size & obstacles
Loader will load the shapely polygon array in an World object
"""


class World:
	"""
	
	"""
	def __init__(self, size=(10,10), obstacles=[], filename = None):

		if filename !=None:
			self.size, self.obstacles = self.load_yaml(filename)
		else:
			self.size = size
			self.obstacles = obstacles

	def load_yaml(self, filename):
		f = open(filename)
		dict_=yaml.safe_load(f)

		size = (dict_['size']['x'], dict_['size']['y'])
		obstacles = [] #NOT IMPLEMENTED YET
		name = dict_['name']

		print("Loaded world: "+name)
		return size, obstacles