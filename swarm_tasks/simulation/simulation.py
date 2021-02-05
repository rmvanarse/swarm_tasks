import swarm_tasks.envs as envs
import swarm_tasks.utils as utils

import numpy as np

DEFAULT_SIZE = (20,20)


class Simulation:

	def __init__(self, size=DEFAULT_SIZE,\
		env=None,\
		num_bots=15,\
		initialization='random'):
	
		self.size = size
		self.env = None #envs.loader.load(env)
		self.swarm = []
		self.num_bots = self.populate(num_bots, initialization)

		print("Initialized simulation with "+str(self.num_bots)+" robots")

	def populate(self, n, initialization):
		"""
		Populates the simulation by spawning Bot objects
		Args:
			n:	number of robots
			initialization
		Returns:
			size of final swarm (self.swarm)
		"""
		for i in range(n):
			x,y,theta = None,None,None

			while(1):
				if initialization=='random':
					x = np.random.rand()*self.size[0]
					y = np.random.rand()*self.size[1]
					theta = np.random.rand()*2*np.pi
				else:
					print("Failed to initialize")

				if self.check_free(x,y,utils.robot.DEFAULT_SIZE):
					break

			self.swarm.append(utils.robot.Bot(x,y,theta))
		
		for bot in self.swarm:
			bot.set_sim(self)

		return len(self.swarm)

	
	def check_free(self, x,y, r):
		"""
		Checks if point (x,y) is free for
		a robot to occupy

		Returns: bool

		ToDo: Check for obstacles in env
		"""
		#Check for borders of simlation
		if (x<r or x>(self.size[0]-r)):
			return False
		if (y<r or y>(self.size[1]-r)):
			return False

		#Check for obstacles in env


		#Check for other bots
		for bot in self.swarm:

			if bot.dist(x,y) < 2*bot.size:
				return False
		return True

