import swarm_tasks

from swarm_tasks.simulation import simulation as sim
from swarm_tasks.simulation import visualizer as viz

import swarm_tasks.utils as utils
import swarm_tasks.envs as envs
import swarm_tasks.controllers as ctrl
import swarm_tasks.controllers.potential_field as potf
import swarm_tasks.controllers.base_control as base_control

from swarm_tasks.modules.aggregation import aggr_centroid, aggr_field
from swarm_tasks.modules.dispersion import disp_field
from swarm_tasks.modules.formations import circle
from swarm_tasks.modules.formations import line	

from swarm_tasks.modules.surround import surround_attractor
from swarm_tasks.modules import exploration as exp

import numpy as np
import multiprocessing
import time


class ControlProcess(multiprocessing.Process):
	def __init__(self, bot):
		multiprocessing.Process.__init__(self)
		self.bot = bot

	def run(self):
		global s
		#global gui

		while 1:
			cmd = base_control.base_control(self.bot)
			cmd+= base_control.obstacle_avoidance(self.bot)
			cmd+=surround_attractor(self.bot)
			cmd += exp.explore(self.bot)

			cmd.exec(self.bot)
			#time.sleep(0.05)
			#gui.update()


#s = sim.Simulation(env_name='empty_world', contents_file='attractors')
s = sim.Simulation(num_bots=15, env_name='rectangles', contents_file='attractors')
gui = viz.Gui(s)

gui.show_env()
gui.show_bots()

for b in s.swarm:
	ControlProcess(b).start()

while 1:
	gui.update()
	#time.sleep(0.05)

