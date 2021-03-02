import swarm_tasks.controllers.potential_field as potf




def surround_attractor(bot, \
			field_weights={'bots':0, 'obstacles':0, 'borders':0, 'goal':0, 'items':-4},\
			order=-1):
	"""
	Surrounds an attractor item within the neighbourhood radius
	ToDo: Tuning
	"""
	cmd = potf.get_field(bot.get_position(), \
		bot.sim, weights=field_weights, \
		order = order, \
		max_dist=bot.neighbourhood_radius,\
		item_types = ['attractor'])


	cmd += potf.get_field(bot.get_position(), \
		bot.sim, weights={'bots':2, 'obstacles':0, 'borders':1, 'goal':-3, 'items':0}, \
		order = order, \
		max_dist=1.2,\
		item_types = ['all'])


	return cmd
