import swarm_tasks.utils as utils
import swarm_tasks.controllers as controllers

import numpy as np

def consensus(bot, include_self=True,\
			neighbourhood_radius=utils.robot.DEFAULT_NEIGHBOURHOOD_VAL):
	"""
	The bot changes its state to the majority state
	in its neighbourhood
	Returns true if the state is switched
	"""
	#Get neighbours
	neighbours = bot.neighbours(neighbourhood_radius)
	if include_self:
		neighbours.append(bot)

	#Count states
	states = [b.get_state() for b in neighbours]
	state_count = [0]*(max(states)+1)

	for s in states:
		state_count[s]+=1

	#Get new state
	new_state = np.argmax(state_count)
	switch_b = not (bot.get_state()==new_state)
	
	bot.set_state(new_state)

	return switch_b


def switch_stoch(bot, new_state, prob=0.05):
	"""
	Switches to the new_state with a fixed probability
	Returns true of the state is switched
	"""

	if np.random.rand()<=prob:
		bot.set_state(new_state)
		return True
	
	return False
