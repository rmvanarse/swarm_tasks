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
	#Count the number of robots surrounding an item
	count = 0

	for bot in sim.swarm:
		pos = Point(bot.get_position())
		p1,p2 = nearest_points(pos, item.polygon)
		r = p2.distance(p1)

		if r<thresh_dist:
			count+=1

	return count


def bots_picking_item(sim, item, pick_up_state=1, thresh_dist = 0.2):
	"""
	List of bots having state=pick_up_state in contact with the item
	"""
	bots = []

	for bot in sim.swarm:
		if bot.state != pick_up_state:
			continue
		pos = Point(bot.get_position())
		p1,p2 = nearest_points(pos, item.polygon)
		r = p2.distance(p1)

		if r<thresh_dist+bot.size:
			bots.append(bot)

	return bots

def item_at_nest(sim, item, nest_pos, nest_size):
	"""
	Check if the item is at the nest
	"""
	if Point(nest_pos).distance(Point(item.pos))<nest_size:
		return True
	else:
		return False


#TODO: Area coverage %check