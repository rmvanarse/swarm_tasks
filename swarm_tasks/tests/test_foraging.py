print("This is a backup script. Please use test_tasks.py")
exit()

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
