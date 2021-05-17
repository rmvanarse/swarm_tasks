import swarm_tasks.utils as utils
import swarm_tasks.controllers as controllers

import numpy as np

def disp_field( bot,\
			field_weights = {'bots':3, 'obstacles':0.5, 'borders':0.5, 'goal':0, 'items':1},\
			neighbourhood_radius = utils.robot.DEFAULT_NEIGHBOURHOOD_VAL,\
			item_types=[]):
	"""
	The robots disperse away from one another and settle down nearly equidistantly
	Uses potential field with a repulsive force from bots, obstacles & items
	The field is defined by a reciprocal law

	Args:
	 - bot
	 - neighbourhood_radius
	 - field_weights: (Highest weight given to robots by defaul, then to obstacles/items)
	 - item_types: Used if dispersion from a specific subset of neighbours is intended
	 				(All neighbours used if list is empty)
	Returns: Cmd for dispersion
	"""
	cmd = controllers.potential_field.get_field(bot.get_position(), \
		bot.sim, weights=field_weights, \
		order = 1, \
		max_dist=neighbourhood_radius, \
		item_types = item_types)

	return cmd
