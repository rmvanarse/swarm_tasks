import swarm_tasks.utils as utils
import swarm_tasks.controllers as controllers

import numpy as np

def aggr_centroid(bot,neighbourhood_radius=utils.robot.DEFAULT_NEIGHBOURHOOD_VAL,\
				single_state=False, state=None):
	"""
	The robot moves towards the centroid of its neighbours
	Args:
		- bot
		- neighbourhood_radius
		- single_state: Set to true if aggrefation is to be performed
						with neighbours of a fixed state
		- state: Used with single_state
	Returns: Cmd for aggregation
	"""

	neighbours = bot.neighbours(neighbourhood_radius, single_state, state)
	num_neighbours = len(neighbours)
	
	if not num_neighbours:
		"""
		(TODO) Enhancement:
		Can define default behaviour for bots without neighbours
		"""
		return controllers.command.Cmd(speed=0, dir_=0)

	centroid = (0,0)
	for n in neighbours:
		x,y = n.get_position()
		centroid = (centroid[0]+x, centroid[1]+y)
		

	centroid = (centroid[0]/num_neighbours, centroid[1]/num_neighbours)

	x0, y0 = bot.get_position()
	dir_vec = np.array([centroid[0]-x0, centroid[1]-y0])
	dir_vec/=np.linalg.norm(dir_vec)

	cmd = controllers.command.Cmd(dir_vec.tolist())

	return cmd


def aggr_field(bot, field_weights = {'bots':-0.5, 'obstacles':0, 'borders':0, 'goal':0}):
	"""
	Uses potential field with an attractive force from neighbourhood robots
	Returns: Cmd for aggregation
	"""
	cmd = controllers.potential_field.get_field(bot.get_position(), \
		bot.sim, weights=field_weights, \
		order = -1, \
		max_dist=bot.neighbourhood_radius)

	return cmd
