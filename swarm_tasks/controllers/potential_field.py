import swarm_tasks.utils as utils
import swarm_tasks.controllers.command as cmd
#import swarm_tasks.simulation as sim

import numpy as np
from shapely.geometry import Point, Polygon
from shapely.ops import nearest_points


field_weights = {'bots':1, 'obstacles':1, 'goal':-1}

def get_field(point, sim, order=2, weights=field_weights, max_dist=4):
	"""
	Args:
		poiint: (x,y) tuple
		sim: Simulation object
		order: r^(-pow)
		weights: weight given to field from bots, obstacles, goal, etc.
	Returns:
		velocity vector as Cmd object
	"""
	vec = np.array([0.0,0.0])

	p = Point(point[0],point[1])
	for o in sim.env.obstacles:
		p1,p2 = nearest_points(p,o)

		r = p2.distance(p1)

		if r>max_dist:
			continue

		dir_vec = -np.array([p2.x-p1.x, p2.y-p1.y])
		dir_vec = np.divide(dir_vec, np.linalg.norm(dir_vec))

		field = field_weights['obstacles']/np.abs(np.power(r+0.001, order))

		vec+=field*dir_vec

	return cmd.Cmd(vec.tolist())		