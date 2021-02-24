import swarm_tasks.controllers.potential_field as potf




def base_control(bot, \
			field_weights={'bots':1, 'obstacles':1, 'borders':1, 'goal':-3},\
			order=2):
	cmd = potf.get_field(bot.get_position(), \
		bot.sim, weights=field_weights, \
		order = order, \
		max_dist=3)

	return cmd


def obstacle_avoidance(bot, \
			field_weights={'bots':1, 'obstacles':1, 'borders':1, 'goal':-3},\
			order=6, k=5):
	cmd = potf.get_field(bot.get_position(), \
		bot.sim, weights=field_weights, \
		order = order, \
		max_dist=1.5)

	return cmd*k
