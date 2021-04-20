import swarm_tasks.utils as utils
import swarm_tasks.controllers.command as cmd
#import swarm_tasks.simulation as sim

import numpy as np
from shapely.geometry import Point, Polygon
from shapely.ops import nearest_points


field_weights = {'bots':1, 'obstacles':1, 'borders':1, 'goal':-3, 'items':0}

def get_field(point, sim, \
	order=2, \
	weights=field_weights, \
	max_dist=2, \
	goal_set = False, goal=None, \
	goal_order = 0,\
	item_types = []):
	"""
	Args:
		poiint: (x,y) tuple
		sim: Simulation object
		order: r^(-pow)
		weights: weight given to field from bots, obstacles, goal, etc.
		max_dist: Max distance of objects (except goal) exerting a field
		goal_set: Bool; whether a goal point has been given
		goal: (x,y) tuple
		goal_order:
		item_types: List containing types of items that generate field 
	Returns:
		velocity vector as Cmd object
	"""
	vec = np.array([0.0,0.0])

	p = Point(point[0],point[1])

	#Obstacle field
	for o in sim.env.obstacles:
		p1,p2 = nearest_points(p,o)

		r = p2.distance(p1)

		if r>max_dist:
			continue

		dir_vec = -np.array([p2.x-p1.x, p2.y-p1.y])
		dir_vec = np.divide(dir_vec, np.linalg.norm(dir_vec))

		field = weights['obstacles']/np.abs(np.power(r+0.001, order))

		vec+=field*dir_vec

	#Robots field
	for b in sim.swarm:
		pos = Point(b.get_position())
		if pos.x == point[0] and pos.y == point[1]:
			continue
		r = pos.distance(p)

		if r>max_dist:
			continue

		dir_vec = -np.array([pos.x-p.x, pos.y-p.y])
		dir_vec = np.divide(dir_vec, np.linalg.norm(dir_vec))

		field = weights['bots']/np.abs(np.power(r+0.001, order))

		vec+=field*dir_vec

	#Boundaries
	#X
	if point[0] < max_dist:
		r = point[0]
		vec[0]+=weights['borders']/np.abs(np.power(r+0.001, order))
	elif point[0] > sim.size[0]-max_dist:
		r = sim.size[0]-point[0]
		vec[0]-=weights['borders']/np.abs(np.power(r+0.001, order))
	#Y
	if point[1] < max_dist:
		r = point[1]
		vec[1]+=weights['borders']/np.abs(np.power(r+0.001, order))
	elif point[1] > sim.size[1]-max_dist:
		r = sim.size[1]-point[1]
		vec[1]-=weights['borders']/np.abs(np.power(r+0.001, order))

	#Goal
	if goal_set:
		r = p.distance(Point(goal[0],goal[1]))
		if r>0.1:
			dir_vec = -np.array([goal[0]-p.x, goal[1]-p.y])
			dir_vec = np.divide(dir_vec, np.linalg.norm(dir_vec))
			field = weights['goal']/np.abs(np.power(r+0.001, goal_order))
			vec+=field*dir_vec


	#Items
	for o in sim.contents.items:
		if not ((o.type in item_types) or ('all' in item_types)):
			continue
		p1,p2 = nearest_points(p,o.polygon)

		r = p2.distance(p1)

		if r>max_dist:
			continue

		dir_vec = -np.array([p2.x-p1.x, p2.y-p1.y])
		dir_vec = np.divide(dir_vec, np.linalg.norm(dir_vec+0.001))

		field = weights['items']/np.abs(np.power(r+0.001, order))

		vec+=field*dir_vec


	return cmd.Cmd(vec.tolist())		