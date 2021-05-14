print("Running foraging example 1\nUsing source files for package imports\nPARAMETERS:")
import sys,os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__),'../../..')))
print(sys.path)

import swarm_tasks

from swarm_tasks.simulation import simulation as sim
from swarm_tasks.simulation import visualizer as viz

import swarm_tasks.utils as utils
import swarm_tasks.envs as envs
import swarm_tasks.controllers as ctrl
import swarm_tasks.controllers.potential_field as potf
import swarm_tasks.controllers.base_control as base_control


from swarm_tasks.tasks import area_coverage as cvg

import numpy as np


s = sim.Simulation(num_bots=10, env_name='rectangles', contents_file='attractors')

gui = viz.Gui(s)
gui.show_env()
gui.show_bots()
gui.show_grid()

while 1:
	for b in s.swarm:

		cmd = cvg.disp_exp_area_cvg(b)
		cmd.exec(b)

	s.update_grid()
	gui.show_grid()
	gui.update()
	s.time_elapsed+=1

gui.run()
