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

		self.state_colors = ['blue','green','red','orange', 'purple']

		self.grid_scatter = None

		#Contents list (used in remove_artists())
		self.content_fills = []
		self.limits = ([0,0,self.size[0], self.size[0]], [0,self.size[1], self.size[1],0])

	def show_bots(self):

		#show bots
		for bot in self.sim.swarm:
			#self.show_neighbourhood(bot,3)

			x,y,theta = bot.get_pose()
			state_disp = (bot.get_state()<=len(self.state_colors))*(bot.get_state())
			circle = plt.Circle((x,y), bot.size, color=self.state_colors[state_disp], fill=True)
			self.fig.gca().add_artist(circle)

			l=0.15
			self.ax.arrow(x,y, \
				(bot.size-l)*np.cos(theta), (bot.size-l)*np.sin(theta), \
				head_width=l, head_length=l, \
				fc='k', ec='k', zorder=100)


	def show_env(self):
		for obs in self.sim.env.obstacles:
			x,y = obs.exterior.xy
			self.ax.fill(x,y, fc='gray', alpha=0.9)

	def show_contents(self):
		for item in self.sim.contents.items:
			x,y = item.polygon.exterior.xy
			if item.subtype == 'contamination':
				self.ax.fill(x,y, fc='red', alpha=0.5)
				continue
			elif item.subtype == 'nest':
				self.ax.fill(x,y, fc='purple', alpha =0.3)
				continue
			self.ax.fill(x,y, fc='orange', alpha=0.9)
			#self.content_fills.append((x,y))
	

	def update(self):
		"""

		"""
		self.remove_artists()
		#self.show_bots()
		if self.sim.has_item_moved:
			#for ext in self.content_fills:
				#self.ax.fill(*ext, fc='w')
			#self.content_fills=[]
			#self.ax.fill(*self.limits, 'w')
			self.show_contents()
			self.show_env()
			self.sim.has_item_moved = False
		self.show_bots()
		plt.pause(0.0005)

	def remove_artists(self):
		"""
		Previous figures need to be removed before updating positions on GUI
		"""
		for obj in self.ax.findobj(plt.Circle):
			obj.remove()

		for obj in self.ax.findobj(patches.FancyArrow):
			obj.remove()
		
		if self.sim.has_item_moved:
			for obj in self.ax.findobj(patches.Polygon):
				obj.remove()


	def show_neighbourhood(self, bot, r=None):
		#Shows the neighbourhood of a robot
		x,y = bot.get_position()
		if r == None:
			r = bot.neighbourhood_radius
		circle2 = plt.Circle((x,y), r, color='red', fill=True, alpha=0.1)
		circle1 = plt.Circle((x,y), r/2+bot.size, color='red', fill=True, alpha=0.1)
		self.fig.gca().add_artist(circle1)
		self.fig.gca().add_artist(circle2)


	def show_grid(self):
		"""
		Args:
			grid: A 2D array of values from 0-1 
		
		TODO:
		Plotting the grid as an image takes high computation
		Plot in another format
		"""
		if self.grid_scatter != None:
			self.grid_scatter.remove()

		x_size, y_size = self.sim.size
		x_labels = [i +0.5 for i in range(x_size)]*y_size
		y_labels = [int(i/x_size)+0.5 for i in range(y_size*x_size)]
		
		grid_1d = self.sim.grid.ravel()
		#grid_1d[20:40] = True	#debug

		col = ['green' if i else 'yellow' for i in grid_1d]


		if len(col) != len(x_labels) or len(x_labels) != len(y_labels):
			print("ERROR: Grid shape mismatch")
			return False

		self.grid_scatter = self.ax.scatter(x_labels, y_labels, c=col)
		#plt.draw()


	def run(self):
		plt.show(block=False)
		