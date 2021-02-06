import numpy as np

DEFAULT_NEIGHBOURHOOD_VAL = 6 #neighbourhood radius
DEFAULT_SIZE = 0.5 #Radius of chassis
MAX_SPEED = 1.5
MAX_ANGULAR = 0.2
DEFAULT_STATE='stop'


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

		#Add to sim?

	
	def dist(self, x, y):
		"""
		Returns the distance of the centre of the robot
		from the point x,y
		"""
		return np.linalg.norm([x-self.x, y-self.y])

	def neighbours(self):
		"""
		Returns the a list of neighbours
		"""
		neighbours = []
		if self.sim == None:
			print("WARNING: Robot is not linked to a simulation")
			return neighbours

		for bot in self.sim.swarm:
			d = bot.dist(self.x, self.y)
			if d>0 and d<self.neighbourhood_radius:
				neighbours.append(bot)

		return neighbours


	def step(self, step_size=0.05):
		x_ = self.x + step_size*np.cos(self.theta)
		y_ = self.y + step_size*np.sin(self.theta)

		if self.sim.check_free(x_,y_,self.size, ignore=self):
			#check_free() will not work right now because
			#a collision with oneself will be detected
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
		turn_angle = direction-self.theta
		while turn_angle>np.pi:
			turn_angle-=2*np.pi
		while turn_angle<-np.pi:
			turn_angle+=2*np.pi

		if np.abs(turn_angle)>self.max_turn_speed:
			self.turn(np.sign(turn_angle)*self.max_turn_speed)
		else:
			self.turn(turn_angle)
			self.step(min(speed,self.max_speed)*step_size)


	def set_sim(self, sim):
		self.sim = sim

	def get_state(self):
		return self.state

	def get_pose(self):
		return self.x, self.y, self.theta

	def get_position(self):
		return self.x, self.y




