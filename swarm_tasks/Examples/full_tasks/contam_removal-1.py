print("Running cont_rem example 1\nUsing source files for package imports\nPARAMETERS:\
	Using default hardcoded parameters (listed in swarm_tasks/logs)")
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
swarm_tasks.utils.robot.MAX_ANGULAR: 1
np.random.seed(42)
random.seed(42)

from swarm_tasks.simulation import simulation as sim
from swarm_tasks.simulation import visualizer as viz
from swarm_tasks.simulation import scenarios


import swarm_tasks.utils as utils

from swarm_tasks.tasks import remove_contamination as remcon

import numpy as np


s = sim.Simulation(num_bots=15, env_name='rectangles')

gui = viz.Gui(s)
gui.show_env()
gui.show_bots()

while 1:
	for b in s.swarm:

		cmd = remcon.remove_contamination(b)
		cmd.exec(b)

	scenarios.contaminations(s, 0.0025, 40) #0.004 for 20 bots

	gui.update()
	s.time_elapsed+=1

gui.run()

