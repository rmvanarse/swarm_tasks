import sys, yaml, os

import shapely
from shapely.geometry import Polygon

import swarm_tasks.utils.item as ex

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
		f = open(filename)
		dict_=yaml.safe_load(f)	
		items = []
		
		if dict_['items']!=None:
			item_list = dict_['items']
			for item in item_list:
				poly, _type = None, None
				try:
					poly = Polygon(item_list[item]['poly'])
					_type = item_list[item]['type']
					assert(_type in ['default', 'attractor', 'clutter'])
				except:
					print("WARNING: Failed to load external item!")
					continue

				if _type == 'default':
					items.append(ex.Item(poly))
				elif _type == 'attractor':
					items.append(ex.Attractor(poly))
				elif _type == 'clutter':
					weight = 6 #Temp
					try:
						weight = item_list[item]['weight']
					except:
						print("No weight specified; Using value "+str(weight))
					items.append(ex.Clutter(poly, weight))
				#obstacles.append(Polygon(o))

		return items