import swarm_tasks.envs as envs
import swarm_tasks.utils as utils

import numpy as np
from shapely.geometry import Point, Polygon

DEFAULT_SIZE = (20,20)


class Simulation:

	def __init__(self, \
		env_name=None,\
		num_bots=15,\
		initialization='random',\
		contents_file=None):

		#Load world into self.env
		if env_name==None:
			self.env = envs.world.World()
			print("Using default world")

		else:
			self.env = envs.world.World(filename=env_name+'.yaml')


		#Set simulation parameters based on world and robots
		self.size = self.env.size
		

		#Load external items on top of env/world
		if contents_file==None:
			self.contents = envs.items.Contents()
			print("No external items loaded")
		else:
			self.contents = envs.items.Contents(filename = contents_file+'.yaml') #Add filename 

		#Populate world with robots
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

	
	def check_free(self, x,y, r, ignore=None):
		"""
		Checks if point (x,y) is free for
		a robot to occupy
		Args:
			x,y: Robot position
			r: Robot radius
			ignore: Robot to ignore 
			(use ignore=self in Bot class to ignore collision with self)
		Returns: bool

		"""
		#Check for borders of simlation
		if (x<r or x>(self.size[0]-r)):
			return False
		if (y<r or y>(self.size[1]-r)):
			return False

		#Check for obstacles in env
		for obs in self.env.obstacles:
			if obs.distance(Point(x,y))<=r:
				return False

		#Check for external items (TODO)
		for item in self.contents.items:
			if item.polygon.distance(Point(x,y))<r:
				return False

		#Check for other bots
		for bot in self.swarm:
			if ignore == bot:
				continue
			if bot.dist(x,y) < 2*bot.size:
				return False
		
		return True


