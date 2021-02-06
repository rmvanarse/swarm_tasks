from swarm_tasks.simulation.simulation import Simulation
import swarm_tasks.envs as envs

import matplotlib.patches as patches
from matplotlib import pyplot as plt
from matplotlib import animation
import numpy as np

class Gui:
	def __init__(self, sim):
		self.sim = sim
		self.size = sim.size

		#self.ax = plt.axes()
		#self.fig = plt.gcf()
		self.fig, self.ax = plt.subplots(figsize=(10,10))
		self.ax.set_ylim([0, sim.size[1]])
		self.ax.set_xlim([0, sim.size[0]])

	def show_all(self):
		#show env

		#show bots
		for bot in self.sim.swarm:
			x,y,theta = bot.get_pose()
			circle = plt.Circle((x,y), bot.size, color='blue', fill=True)
			self.fig.gca().add_artist(circle)
			

			l=0.15
			self.ax.arrow(x,y, \
				(bot.size-l)*np.cos(theta), (bot.size-l)*np.sin(theta), \
				head_width=l, head_length=l, \
				fc='k', ec='k')

	def show_env(self):
		for obs in self.sim.env.obstacles:
			x,y = obs.exterior.xy
			self.ax.fill(x,y, fc='r', alpha=0.5)

	def update(self):
		"""
		Not in use atm,
		Use animate instead
		"""
		self.remove_artists()
		self.show_all()
		plt.pause(0.05)

	def remove_artists(self):
		for obj in self.ax.findobj(plt.Circle):
			obj.remove()

		for obj in self.ax.findobj(patches.FancyArrow):
			obj.remove()


	def show_neighbourhood(self, bot):
		x,y = bot.get_position()
		circle = plt.Circle((x,y), bot.neighbourhood_radius, color='red', fill=False)
		self.fig.gca().add_artist(circle)



	def run(self):
		plt.show()
		