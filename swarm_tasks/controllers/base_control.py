import swarm_tasks.controllers.potential_field as potf




def base_control(bot, \
			field_weights={'bots':1, 'obstacles':1, 'borders':0.5, 'goal':-3, 'items':0},\
			order=2):
	"""
	ToDo: Tuning
	"""
	cmd = potf.get_field(bot.get_position(), \
		bot.sim, weights=field_weights, \
		order = order, \
		max_dist=3)

	return cmd


def obstacle_avoidance(bot, \
			field_weights={'bots':1, 'obstacles':1, 'borders':0.5, 'goal':-3, 'items':0.05},\
			order=4, k=3):
	"""
	ToDo: Tuning
	"""
	cmd = potf.get_field(bot.get_position(), \
		bot.sim, weights=field_weights, \
		order = order, \
		max_dist=bot.size+0.4,\
		item_types = ['all'])

	return cmd*k
