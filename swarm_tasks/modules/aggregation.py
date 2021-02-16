import swarm_tasks.utils as utils
import swarm_tasks.controllers as controllers

import numpy as np

def aggr_centroid(bot):

	neighbours = bot.neighbours()
	num_neighbours = len(neighbours)
	
	if not num_neighbours:
		"""
		Enhancement:
		Can define default behaviour for bots without neighbours
		"""
		return controllers.command.Cmd(speed=0, dir_=0)

	centroid = (0,0)
	for n in neighbours:
		x,y = n.get_position()
		centroid = (centroid[0]+x, centroid[1]+y)
		
		num_neighbours+=1

	centroid = (centroid[0]/num_neighbours, centroid[1]/num_neighbours)

	x0, y0 = bot.get_position()
	dir_vec = np.array([centroid[0]-x0, centroid[1]-y0])
	dir_vec/=np.linalg.norm(dir_vec)

	cmd = controllers.command.Cmd(dir_vec.tolist())

	return cmd