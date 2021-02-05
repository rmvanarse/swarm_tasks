from swarm_tasks.simulation.simulation import Simulation
import swarm_tasks.envs as envs

from matplotlib import pyplot as plt

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

		plt.show()

	def update(self):
		pass
