print("Running area coverage example 1\nUsing source files for package imports\nPARAMETERS:\
	Using default hardcoded weight_dicts (listed in swarm_tasks/logs)")
import sys,os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__),'../../..')))
print(sys.path)

import swarm_tasks

#Set demo parameters directly
import numpy as np
import random
swarm_tasks.utils.robot.DEFAULT_NEIGHBOURHOOD_VAL = 6
swarm_tasks.utils.robot.DEFAULT_SIZE= 0.4
swarm_tasks.utils.robot.MAX_SPEED = 1.5
swarm_tasks.utils.robot.MAX_ANGULAR: 1.0
np.random.seed(42)
random.seed(42)

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
