import swarm_tasks.envs as envs
import swarm_tasks.utils as utils

import numpy as np
import pandas as pd
import random
from shapely.geometry import Point, Polygon

DEFAULT_SIZE = (20,20)


class Simulation:

	def __init__(self, \
		env_name=None,\
		num_bots=15,\
		initialization='random',\
		num_initial_states = 1,\
		contents_file=None,\
		init_neighbourhood_radius=utils.robot.DEFAULT_NEIGHBOURHOOD_VAL):

		#Load world into self.env
		if env_name==None:
			self.env = envs.world.World()
			self.env_name = "Default"
			print("Using default world")

		else:
			self.env = envs.world.World(filename=env_name+'.yaml')
			self.env_name = env_name


		#Set simulation parameters based on world and robots
		self.size = self.env.size
		

		#Load external items on top of env/world
		if contents_file==None:
			self.contents = envs.items.Contents()
			self.contents_name = "None"
			print("No external items loaded")
		else:
			self.contents = envs.items.Contents(filename = contents_file+'.yaml') #Add filename 
			self.contents_name = contents_file
		
		self.has_item_moved = True #(Used to decide whether to update polygons in viz)
		

		#Grid
		self.grid = np.zeros(self.env.size, dtype=bool)	#Default grid


		#Populate world with robots
		self.swarm = []
		self.num_initial_states = num_initial_states
		self.nr = init_neighbourhood_radius
		self.num_bots = self.populate(num_bots, initialization, \
									self.num_initial_states,self.nr)

		#Other instance variables:
		self.time_elapsed = 0	#Number of iterations
		self.state_list = None
		

		print("Initialized simulation with "+str(self.num_bots)+" robots")
		

	def populate(self, n, initialization, num_states, nr):
		"""
		Populates the simulation by spawning Bot objects
		Args:
			n:	number of robots
			initialization
			num_states
			nr: neighbourhood radius of the bots
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

					state = random.randint(0,num_states-1)
				else:
					print("Failed to initialize")

				if self.check_free(x,y,utils.robot.DEFAULT_SIZE):
					break

			self.swarm.append(utils.robot.Bot(x,y,theta, state=state, neighbourhood_radius=nr))
		
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
			if ignore == item:
				continue
			if item.polygon.distance(Point(x,y))<=r:
				return False

		#Check for other bots
		for bot in self.swarm:
			if ignore == bot:
				continue
			if bot.dist(x,y) < 2*bot.size:
				return False
		
		return True

	
	def create_custom_grid(self, size, _type='bool'):
		self.grid = np.zeros(size, dtype=_type)
		return True

	def update_grid(self, r=1, new_val=True):
		"""
		Updates grid values in areas of radius r around all bots
		(For area coverage) 
		The value cahnges to new_val

		Returns the fraction of True values for info
		"""
		for bot in self.swarm:
			x,y = bot.get_position()
			self.grid[max(0,int(y-r)):min(int(y+r+1),self.grid.shape[0]),\
			max(0,int(x-r)):min(int(x+r+1),self.grid.shape[1])] = True
		
		return np.sum(self.grid)/(self.grid.size)

	#LOG THE SIMULATION PARAMETERS (Not the same as save sim)
	def get_sim_param_log(self):
		
		params = str()
		params += "# SIMULATION PARAMETERS\n"
		params += "\nsize: " + str(self.size)
		params += "\nnum_bots: " + str(self.num_bots)
		params += "\nenv_name: " + str(self.env_name)
		params += "\ncontents_name: " + str(self.contents_name)

		return params + "\n\n"

	#FOR LOADED SIMULATIONS:
	#The visualizer should work for loaded sims without modifications

	def save_state(filename):
		"""
		Appends the state (x,y,theta) of each robot in the swarm
		to a csv file (filename)
		"""
		return True

	def save_sim(filename):
		"""
		Creates a yaml file (filename.yaml) with the simulation parameters
		i.e. num_bots, size, obstacles file, items, states_file, etc.
		Also creates an empty csv file (states_file) to save states

		"""
		return True

	def load_sim(filename):
		"""
		Loads a simulation (env, items, etc.) from its yaml file
		Loads the first state (if exists) from the corresponding csv file
		Loads the state_list using sim.create)state_list
		"""
		return True

	def load_state(state_array):
		"""
		Changes robot states to the states from the paramter array
		"""
		return True

	def create_state_list(filename):
		"""
		Reads the csv file and creates a list of states
		for the simulation to load one by one

		Returns: Pandas dataframe
		TODO: Can this be done better without a DF?
		"""

		return state_list


