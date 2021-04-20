import swarm_tasks.utils as utils
import swarm_tasks.controllers as controllers

import numpy as np

def follow_leader(bot, leader):
	"""
	Returns a Cmd object in the direction of the 'leader'
	Note: Each bot can have a different leader
	'leader' is not a global term
	"""
	x0,y0 = bot.get_position()
	x1,y1 = leader.get_position()

	dir_vec = np.array([x1-x0, y1-y0])
	dir_vec/=np.linalg.norm(dir_vec)

	cmd = controllers.command.Cmd(dir_vec.tolist())

	return cmd