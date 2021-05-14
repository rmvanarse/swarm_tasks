print("Running foraging example 1\nUsing source files for package imports\nPARAMETERS:")
import sys,os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__),'../../..')))
print(sys.path)

import swarm_tasks

from swarm_tasks.simulation import simulation as sim
from swarm_tasks.simulation import visualizer as viz
from swarm_tasks.simulation import scenarios


import swarm_tasks.utils as utils
import swarm_tasks.controllers.base_control as base_control

from swarm_tasks.tasks import foraging as frg


s = sim.Simulation(num_bots=20, env_name='empty_world')

gui = viz.Gui(s)
gui.show_env()
gui.show_bots()

while 1:
	for b in s.swarm:

		cmd = frg.gather_resources(b)
		cmd.exec(b)

	scenarios.movable_resources(s,5)

	gui.update()
	s.time_elapsed+=1

gui.run()
