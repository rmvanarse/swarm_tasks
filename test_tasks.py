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

import swarm_tasks.utils.logging as logging

def save_log(sim):
	robot_params = logging.log_robot()
	sim_params = sim.get_sim_param_log()
	event_logs = scenarios.get_event_log()
	logging.save_log([robot_params,sim_params,event_logs], "swarm_tasks/logs", "TEST_LOGS_CONTAMINATION_REMOVAL-", ext=".yaml")


#s = sim.Simulation(env_name='empty_world', contents_file='attractors')
#s = sim.Simulation(num_bots=10, env_name='rectangles', contents_file='attractors')
#s = sim.Simulation(num_bots=20, env_name='empty_world')

s = sim.Simulation(num_bots=15, env_name='rectangles')

gui = viz.Gui(s)
gui.show_env()
gui.show_bots()

while 1:
	for b in s.swarm:

		cmd = remcon.remove_contamination(b)
		cmd.exec(b)

	scenarios.contaminations(s, 0.003, 40) #0.004 for 20 bots

	gui.update()
	s.time_elapsed+=1

	save_log(s)

gui.run()


