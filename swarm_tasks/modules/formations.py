import swarm_tasks.utils as utils
import swarm_tasks.controllers as controllers

import numpy as np

def circle(bot, radius):
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
		

	centroid = (centroid[0]/num_neighbours, centroid[1]/num_neighbours)

	x0, y0 = bot.get_position()
	dir_vec = np.array([centroid[0]-x0, centroid[1]-y0])
	dir_vec/=np.linalg.norm(dir_vec)

	dir_vec*=(bot.dist(centroid[0], centroid[1])-radius)
	dir_vec/=np.linalg.norm(dir_vec)

	cmd = controllers.command.Cmd(dir_vec.tolist())

	return cmd


def line(bot):
	"""
	(**Trial basis**)
	Performs linear regression among neighbours
	Returns cmd towards (& perpendicular) to the line
	"""
	neighbours = bot.neighbours()
	num_neighbours = len(neighbours)

	if not num_neighbours:
		"""
		Enhancement:
		Can define default behaviour for bots without neighbours
		"""
		return controllers.command.Cmd(speed=0, dir_=0)

	X = []
	Y = []
	x0, y0 = bot.get_position()
	#Find line y=mx+c
	for n in neighbours:
		x,y = n.get_position()
		X.append(x)
		Y.append(y)

	X = np.asarray(X)
	Y = np.asarray(Y)
	C = np.ones(X.shape)
	A = np.vstack([X,C]).T#.reshape(num_neighbours,2)


	m,c = np.linalg.lstsq(A,Y, rcond=None)[0]
	print(m,c)

	#Get direction of cmd
	dir_ = np.arctan(-1/m)
	speed = -np.sign(m)*(m*x0 + c- y0)	;


	cmd = controllers.command.Cmd(speed=speed, dir_=dir_) #temp
	return cmd