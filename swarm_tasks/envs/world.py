import sys, yaml, os
import swarm_tasks.envs.world

import shapely
from shapely.geometry import Polygon
"""
Each env is stored as a yaml file listing size & obstacles
Loader will load the shapely polygon array in an World object
"""

#Relative paths (May cause complications while running from different folders)
envs_path = os.path.dirname(os.path.abspath(__file__))
worlds_path = os.path.join(envs_path, 'worlds')

class World:
	"""
	An instance of this is loaded in th simulation
	A world is defined using dimensions and a list of obstcles
	"""
	def __init__(self, size=(20,20), obstacles=[], filename = None):

		if filename !=None:
			file = os.path.join(worlds_path, filename)
			
			self.size, self.obstacles = self.load_yaml(file)
		else:
			self.size = size
			self.obstacles = obstacles

	def load_yaml(self, filename):
		f = open(filename)
		dict_=yaml.safe_load(f)
		obstacles = []
		size = (dict_['size']['x'], dict_['size']['y'])
		
		if dict_['obstacles']!=None:
			obstacle_list = dict_['obstacles']
			for o in obstacle_list:
				obstacles.append(Polygon(o))

		name = dict_['name']

		print("Loaded world: "+name)
		return size, obstacles