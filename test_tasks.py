import swarm_tasks

from swarm_tasks.simulation import simulation as sim
from swarm_tasks.simulation import visualizer as viz
from swarm_tasks.simulation import scenarios


import swarm_tasks.utils as utils
import swarm_tasks.envs as envs
import swarm_tasks.controllers as ctrl
import swarm_tasks.controllers.potential_field as potf
import swarm_tasks.controllers.base_control as base_control

from swarm_tasks.modules.surround import surround_attractor
from swarm_tasks.tasks import area_coverage as cvg
from swarm_tasks.tasks import remove_contamination as remcon
from swarm_tasks.tasks import foraging as frg
import numpy as np


#s = sim.Simulation(env_name='empty_world', contents_file='attractors')
s = sim.Simulation(num_bots=20, env_name='empty_world')
#s = sim.Simulation(num_bots=20, env_name='rectangles')

gui = viz.Gui(s)
gui.show_env()
gui.show_bots()
#gui.show_grid()

while 1:
	for b in s.swarm:

		cmd = frg.gather_resources(b)
		cmd.exec(b)

	scenarios.movable_resources(s,5)

	#s.update_grid()
	#gui.show_grid()
	gui.update()
	s.time_elapsed+=1

gui.run()
