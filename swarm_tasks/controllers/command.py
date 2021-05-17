import numpy as np
import swarm_tasks.simulation
from swarm_tasks.utils import robot

"""
(1)
Different components will exert a 'force vector' on the robot
Effective direction and speed will be decided by the resultant of all force vectors

(2)
Each controller will return the next direction & speed to command
(uses an object of the Cmd class)

(3)
Low-range potential field will be in effect continuously
Control functions for tasks, PID, etc. will add to low-range potential field

"""

VEC_MAX = robot.MAX_SPEED*3.5	#For truncation

class Cmd:
	"""
	Output of the controller
	"""
	def __init__(self, vec=[], speed=1, dir_=None):
		if len(vec)==2:
			#Initialize using vector
			if dir_!=None:
				print("Vector suplied explicitly; Ignoring dir_...\n")
			self.vec = np.array([vec[0], vec[1]])*speed
			self.dir = np.arctan(float(vec[1])/(float(vec[0])+0.00001))
			
			if vec[0] < 0:
				self.dir += np.pi
				#Needed because arctan codomain is [-pi/2,pi/2]
				
			self.speed = np.linalg.norm(self.vec)
		else:
			#Initialize using direction and speed
			self.speed = speed
			if dir_==None:
				dir_=0
			self.dir = dir_

			while self.dir<=-np.pi:
				self.dir+=2*np.pi
			while self.dir>=np.pi:
				self.dir-=2*np.pi

			self.vec = np.array([np.cos(self.dir), np.sin(self.dir)])*self.speed
			#self.trunc(VEC_MAX)
	
	def trunc(self, max_val):
		#Trucate output (Not in use)
		self.vec*=min(max_val/(self.speed+0.00001), 1)
		self.speed = min(self.speed, max_val)
		

	def exec(self, bot):
		#Execute the command on a robot
		bot.move(self.dir, self.speed)

	def __add__(self, cmd):
		return Cmd((self.vec + cmd.vec).tolist())

	def __mul__(self, k):
		return Cmd((k*self.vec).tolist())
