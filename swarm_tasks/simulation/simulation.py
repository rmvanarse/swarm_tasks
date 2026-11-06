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
		num_initial_states = 1,\
		contents_file=None,\
		init_neighbourhood_radius=utils.robot.DEFAULT_NEIGHBOURHOOD_VAL,\
		vehicle_type=utils.robot.DEFAULT_VEHICLE_TYPE,\
		speed=None,\
		min_speed=None,\
		max_turn_speed=None,\
		bank_angle_deg=None,\
		world_size=None,\
		world_width=None,\
		world_height=None,\
		world_resolution=None):
		#vehicle_type: a single string ('fixedwing'/'ground'), applied to
		#every bot (original behaviour), OR a list with one entry per bot
		#-- index-aligned with whatever `positions`/num_bots populate()
		#is later given -- for a hybrid swarm mixing copters and
		#fixed-wing planes in one run (e.g. vehicle.py's per-drone
		#auto-detected types). Per-bot kinematic defaults (fixedwing:
		#20m/s cruise, 30deg max bank, near-constant airspeed; ground:
		#the original omni defaults) are resolved individually for each
		#bot in populate() -- see _resolve_kinematics() -- since a mixed
		#swarm can't share a single set of defaults.

		#Load world into self.env. Size is fully configurable -- pass
		#world_size=(w,h), or world_width=/world_height= separately, at
		#any scale (metres): a 20x20m test world and a 10km x 10km
		#mission area go through exactly the same code path here.
		#world_resolution (metres/cell) controls only the granularity of
		#per-cell bookkeeping (see envs/coverage_grid.py) -- it never
		#changes how movement, Bezier paths, or swarm logic behave,
		#since those all work in continuous (x,y) metres, not grid cells.
		if env_name==None:
			self.env = envs.world.World(size=world_size, width=world_width, \
				height=world_height, resolution=world_resolution)
			self.env_name = "Default"
			print("Using default world, size="+str(self.env.size)+\
				", resolution="+str(self.env.resolution))

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
		

		#Coverage grid: dense for small/medium worlds, automatically
		#falls back to a sparse (visited-cell) backend for large ones,
		#so this never allocates memory proportional to world size --
		#see envs/coverage_grid.py.
		self.grid = envs.coverage_grid.CoverageGrid(self.env)


		#Populate world with robots
		self.swarm = []
		self.num_initial_states = num_initial_states
		self.nr = init_neighbourhood_radius
		self.vehicle_type = vehicle_type
		self.speed = speed
		self.min_speed = min_speed
		self.max_turn_speed = max_turn_speed
		self.bank_angle_deg = bank_angle_deg
		self.num_bots = 0
		# if positions is not None:
		# 	initialization = 'explicit'
		# self.num_bots = self.populate(num_bots, initialization, \
									# self.num_initial_states,self.nr, positions)
		#Other instance variables:
		self.time_elapsed = 0	#Number of iterations
		self.state_list = None
		print("Swarm Class Initialized")
  
	def depopulate(self):
		self.swarm = []
		self.num_bots = 0
		print("Swarm depopulated: 0 bots remaining")

	@staticmethod
	def _per_bot_value(value, i):
		"""value[i] if value is a per-bot list/tuple, else value
		unchanged (applied uniformly to every bot). Same convention
		for every field that can vary per-bot in a hybrid swarm --
		currently vehicle_type and speed.
		"""
		return value[i] if isinstance(value, (list, tuple)) else value

	@staticmethod
	def _check_per_bot_length(name, value, n):
		if isinstance(value, (list, tuple)) and len(value) != n:
			raise ValueError(
				"%s list must have exactly num_bots=%d entries (one per "
				"bot) -- got %d" % (name, n, len(value)))

	@staticmethod
	def _resolve_kinematics(vehicle_type, speed, min_speed, max_turn_speed, bank_angle_deg):
		"""Per-bot kinematic defaults for vehicle_type ('fixedwing' or
		'ground'), filling in only whatever wasn't explicitly given --
		same rule __init__ used to apply once for the whole (uniform)
		swarm, now applied per-bot so a hybrid swarm gets each bot its
		own type-appropriate defaults.
		"""
		if vehicle_type == 'fixedwing':
			if speed is None:
				speed = utils.robot.DEFAULT_FIXEDWING_SPEED
			if bank_angle_deg is None and max_turn_speed is None:
				bank_angle_deg = utils.robot.DEFAULT_BANK_ANGLE_DEG
			if min_speed is None:
				min_speed = speed
		else:
			if speed is None:
				speed = utils.robot.MAX_SPEED
			if min_speed is None:
				min_speed = utils.robot.DEFAULT_MIN_SPEED
			if max_turn_speed is None:
				max_turn_speed = utils.robot.MAX_ANGULAR
		return speed, min_speed, max_turn_speed, bank_angle_deg

	def _populate(self,positions,num_bots):
		self.depopulate()
		if positions is not None:
			initialization = 'explicit'
		self.num_bots = self.populate(num_bots, initialization, \
									self.num_initial_states,self.nr, positions)
		print("Initialized simulation with "+str(self.num_bots)+" robots")
  
	def populate(self, n, initialization, num_states, nr, positions=None):
		"""
		Populates the simulation by spawning Bot objects
		Args:
			n:	number of robots
			initialization: 'random' (default) spawns each bot at a
				random free point in the world. 'explicit' spawns each
				bot at a caller-given position -- see `positions`.
			positions: required when initialization=='explicit'. A list
				of n (x, y) or (x, y, theta) tuples, one per bot, in
				world units (metres) -- theta (heading, radians) is
				optional per-entry and defaults to 0 if omitted. Ignored
				for initialization=='random'.
			num_states: number of possible initial internal states
			nr: neighbourhood radius of the bots
		Returns:
			size of final swarm (self.swarm)
		"""
		if initialization == 'explicit':
			if positions is None or len(positions) != n:
				raise ValueError(
					"initialization='explicit' requires `positions` with "
					"exactly num_bots=%d (x,y) or (x,y,theta) entries -- got %s" \
					% (n, "None" if positions is None else len(positions)))

		#vehicle_type, speed and bank_angle_deg can each be given as a
		#single value (applied to every bot) or a per-bot list/tuple --
		#e.g. vehicle.py's auto-detected per-drone vehicle_type, real
		#configured cruise speed and (fixedwing) max bank angle, for a
		#hybrid swarm where copters and fixedwings are independently
		#configured and genuinely differ. A list must cover every bot
		#about to be spawned -- there's no sensible default for
		#whichever entries would otherwise be missing.
		self._check_per_bot_length('vehicle_type', self.vehicle_type, n)
		self._check_per_bot_length('speed', self.speed, n)
		self._check_per_bot_length('bank_angle_deg', self.bank_angle_deg, n)

		for i in range(n):
			x,y,theta = None,None,None

			if initialization == 'explicit':
				# Real vehicle positions (swarm.py repopulates every bot
				# from the live GPS-converted x,y every time a mission
				# starts), not a free spot being picked WITHIN the sim's
				# own bounded canvas. A real drone's (x,y) can legitimately
				# be negative or past world_size (e.g. anything south/west
				# of whatever origin was chosen) or sit over ground the
				# sim treats as an obstacle polygon -- world_size/obstacles
				# describe the sim's own visualization/coverage-grid
				# canvas, they are not a constraint on where a real
				# aircraft is allowed to be. The bot's job is to track
				# wherever the vehicle actually is, never to reject it.
				p = positions[i]
				x, y = float(p[0]), float(p[1])
				theta = float(p[2]) if len(p) > 2 else 0.0
				state = random.randint(0,num_states-1)
			else:
				while(1):
					if initialization=='random':
						x = np.random.rand()*self.size[0]
						y = np.random.rand()*self.size[1]
						theta = np.random.rand()*2*np.pi

						state = random.randint(0,num_states-1)
					else:
						print("Failed to initialize")
						break

					if self.check_free(x,y,utils.robot.DEFAULT_SIZE):
						break

			bot_vehicle_type = self._per_bot_value(self.vehicle_type, i)
			bot_speed_override = self._per_bot_value(self.speed, i)
			bot_bank_angle_override = self._per_bot_value(self.bank_angle_deg, i)
			bot_speed, bot_min_speed, bot_max_turn_speed, bot_bank_angle_deg = \
				self._resolve_kinematics(bot_vehicle_type, bot_speed_override, \
					self.min_speed, self.max_turn_speed, bot_bank_angle_override)

			self.swarm.append(utils.robot.Bot(x,y,theta, state=state, neighbourhood_radius=nr,\
				vehicle_type=bot_vehicle_type, speed=bot_speed,\
				min_speed=bot_min_speed, max_turn_speed=bot_max_turn_speed,\
				bank_angle_deg=bot_bank_angle_deg))
		
		for bot in self.swarm:
			bot.set_sim(self)

		return len(self.swarm)

	def update_bots_location(self,bot_index,x,y,theta):
		bot = self.swarm[bot_index]
		bot.x = x
		bot.y = y
		bot.theta = theta
	
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
		"""
		Legacy override: replaces the coverage grid with a raw dense
		numpy array of the given (rows, cols) shape, bypassing the
		world-resolution-based CoverageGrid entirely. Kept for backward
		compatibility with any code that indexed self.grid directly;
		prefer passing world_resolution to Simulation() instead, which
		keeps the dense/sparse scaling behaviour of CoverageGrid.
		"""
		self.grid = np.zeros(size, dtype=_type)
		return True

	def update_grid(self, r=1, new_val=True):
		"""
		Updates grid values in areas of radius r (world units, e.g.
		metres) around all bots (for area coverage).

		Returns the fraction of covered/True cells for info. Works
		whether self.grid is the default CoverageGrid (dense or sparse,
		any world size) or a raw ndarray from create_custom_grid().
		"""
		if hasattr(self.grid, 'mark_bots'):
			self.grid.mark_bots(self.swarm, r=r)
			return self.grid.fraction_covered()

		#Legacy raw-ndarray grid: row=y, col=x, consistent with
		#World.to_cell()/CoverageGrid.
		for bot in self.swarm:
			x,y = bot.get_position()
			self.grid[max(0,int(y-r)):min(int(y+r+1),self.grid.shape[0]),\
			max(0,int(x-r)):min(int(x+r+1),self.grid.shape[1])] = new_val

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

