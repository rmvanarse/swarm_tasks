import numpy as np

DEFAULT_NEIGHBOURHOOD_VAL = 6 #neighbourhood radius
DEFAULT_SIZE = 0.4 #Radius of chassis
MAX_SPEED = 1.5
MAX_ANGULAR = 0.3
DEFAULT_STATE=0


class Bot:
	"""
	(Update)
	"""
	def __init__(self, x, y, theta=0,\
		state=DEFAULT_STATE,\
		size=DEFAULT_SIZE,\
		neighbourhood_radius=DEFAULT_NEIGHBOURHOOD_VAL,\
		speed=MAX_SPEED, \
		max_turn_speed=MAX_ANGULAR, \
		verbose=False):

		#TODO: Bot ID 

		self.x = x
		self.y = y
		self.theta = theta
		self.state = state
		self.size = size
		self.neighbourhood_radius = neighbourhood_radius
		self.max_speed = speed
		self.max_turn_speed = max_turn_speed
		self.verbose=verbose
		if(verbose):
			print("New bot spawned at: ("+str(x)+','+str(y)+')')

		self.sim = None #Reference to Simulation object that spawns the bot

		#Goal (weight given to goal in potential field)
		self.goal_given = False
		self.goal = [0,0]

		#For exploration
		self.explore_dir = np.pi*(2*np.random.rand()-1)

		#Add to sim?

	
	def dist(self, x, y):
		"""
		Returns the distance of the centre of the robot
		from the point x,y
		"""
		return np.linalg.norm([x-self.x, y-self.y])

	def neighbours(self, radius=None, single_state=False, state=None):
		"""
		Returns the a list of neighbours

		If sengle_state is true, then return only neighbours of the given state

		Enhancement:
		(2) Include self
		"""
		if radius == None:
			radius = self.neighbourhood_radius

		neighbours = []
		if self.sim == None:
			print("WARNING: Robot is not linked to a simulation")
			return neighbours

		for bot in self.sim.swarm:
			#Skip if a single state is needed
			if single_state:
				if bot.state != state:
					continue
			
			#Get distance to bot
			d = bot.dist(self.x, self.y)
			
			#Check if neighbour
			if d>0 and d<radius:
				neighbours.append(bot)

		return neighbours


	def step(self, step_size=0.05):
		x_ = self.x + step_size*np.cos(self.theta)
		y_ = self.y + step_size*np.sin(self.theta)

		if self.sim.check_free(x_,y_,self.size-0.01, ignore=self):
			self.x, self.y = x_,y_
		else:
			print("Collision!")

	def turn(self, angle):
		self.theta +=angle
		while self.theta>2*np.pi:
			self.theta-=2*np.pi
		while self.theta<0:
			self.theta+=2*np.pi

	def move(self, direction, speed, step_size=0.05):
		"""
		The robot moves one step in the given direction
		The size of step os step_size*speed
		The speed used is the minimum of speed param and self.max_speed
		The bot does not start moving till the direction is within max_turn_speed
		"""
		turn_angle = direction-self.theta
		while turn_angle>=np.pi:
			turn_angle-=2*np.pi
		while turn_angle<=-np.pi:
			turn_angle+=2*np.pi

		if np.abs(turn_angle)>self.max_turn_speed:
			#If direction-theta is large, only turn without moving
			self.turn(np.sign(turn_angle)*self.max_turn_speed)
		else:
			self.turn(turn_angle)
			self.step(min(speed,self.max_speed)*step_size)


	def set_sim(self, sim):
		self.sim = sim

	def set_goal(self, x, y):
		self.goal = (x,y)
		self.goal_given = True
		#self.state=?

	def cancel_goal(self):
		self.goal_given = False

	def goal_exists(self):
		return self.goal_given


	def get_state(self):
		return self.state

	def set_state(self, state):
		self.state = state

	def get_pose(self):
		return self.x, self.y, self.theta

	def get_position(self):
		return self.x, self.y

	def get_dir(self):
		return self.theta




