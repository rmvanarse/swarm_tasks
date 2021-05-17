import swarm_tasks.utils as utils
import swarm_tasks.controllers as controllers

import numpy as np


def explore(bot, prob_switch = 0.0066, speed=5):
	"""
	The robot keeps moving in its current direction with a probability (prob_switch) of
	changing its direction of motion
	For changing direction, a new direction is chosen randomly from a uniform distrbution
	
	Reuturns: Cmd
	
	TODO:
		-Bot State 
		-Tune
	"""

	if np.random.rand()<prob_switch:
		bot.explore_dir = (2*np.random.rand()-1)*np.pi


	return controllers.command.Cmd(speed = speed, dir_= bot.explore_dir)

	