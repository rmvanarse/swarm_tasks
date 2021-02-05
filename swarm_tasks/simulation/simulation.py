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

				if self.check_free(x,y):
					break

			self.swarm.append(utils.robot.Bot(x,y,theta))
		return len(self.swarm)

	
	def check_free(self, x,y):
		"""
		Checks if point (x,y) is free for
		a robot to occupy
		"""

		for bot in self.swarm:
			#Check for borders of simlation
			if (x<bot.size or x>self.size[0]-bot.size):
				return False
			if (y<bot.size or y>self.size[1]-bot.size):
				return False

			#Check for obstacles in env


			#Check for other bots
			if bot.dist(x,y) < bot.size:
				return False
		return True

