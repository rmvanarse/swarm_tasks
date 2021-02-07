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

class Cmd:
	"""
	Output of the controller
	"""
	def __init__(self, vec=None, speed=1, dir_=None):
		if vec!=None:
			#Initialize using vector
			if dir_!=None:
				print("Vector suplied explicitly; Ignoring dir_...\n")
			self.vec = np.array([vec[0], vec[1]])*speed
			self.dir = np.arctan(np.divide(vec[1], vec[0]))
			self.speed = np.norm(self.vec)
		else:
			#Initialize using direction and speed
			self.speed = speed
			if dir_==None:
				dir_=0
			self.dir = dir_

			while self.dir<-np.pi:
				self.dir+=2*np.pi
			while self.dir>np.pi:
				self.dir-=2*np.pi

			self.vec = np.array([np.cos(self.dir), np.sin(self.dir)])*self.speed

	def exec(self, bot):
		bot.move(self.dir, self.speed)

	def __add__(self, cmd):
		return Cmd(self.vec + cmd.vec)

	def __mul__(self, k):
		return Cmd(k*self.vec)