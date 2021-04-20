import swarm_tasks.utils as utils
import swarm_tasks.controllers as controllers

import numpy as np


def explore(bot, prob_switch = 0.0066, speed=5):
	"""
	ToDo:
		-Bot State 
		-Tune
	"""

	if np.random.rand()<prob_switch:
		bot.explore_dir = (2*np.random.rand()-1)*np.pi


	return controllers.command.Cmd(speed = speed, dir_= bot.explore_dir)

	