print("Running foraging example 1\nUsing source files for package imports\nPARAMETERS:")
import sys,os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__),'../../..')))
print(sys.path)

import swarm_tasks

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

