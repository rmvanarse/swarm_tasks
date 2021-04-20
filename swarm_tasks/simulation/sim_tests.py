"""
This file contains different tests that can be conducted in the simulation
Actions taken by the simulation as a response to these tests are also included
"""
import swarm_tasks.utils as utils
#import swarm_tasks.simulation as sim

import numpy as np
from shapely.geometry import Point, Polygon
from shapely.ops import nearest_points

#ITEMS

def num_bots_around_item(sim, item, thresh_dist = 0.2):

	count = 0

	for bot in sim.swarm:
		pos = Point(bot.get_position())
		p1,p2 = nearest_points(pos, item.polygon)
		r = p2.distance(p1)

		if r<thresh_dist:
			count+=1

	return count