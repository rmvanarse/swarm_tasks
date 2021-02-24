import swarm_tasks.utils as utils
import swarm_tasks.controllers as controllers

import numpy as np

def disp_field(bot, field_weights = {'bots':3, 'obstacles':0.5, 'borders':0.5, 'goal':0}):
	"""
	ToDo: Tune parameters
	"""
	cmd = controllers.potential_field.get_field(bot.get_position(), \
		bot.sim, weights=field_weights, \
		order = 1, \
		max_dist=bot.neighbourhood_radius)

	return cmd