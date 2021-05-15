print("Running foraging example 1\nUsing source files for package imports\nPARAMETERS:")
import sys,os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__),'../../..')))
print(sys.path)

import swarm_tasks

#Set demo parameters directly
import numpy as np
import random
swarm_tasks.utils.robot.DEFAULT_NEIGHBOURHOOD_VAL = 12
swarm_tasks.utils.robot.DEFAULT_SIZE= 0.4
swarm_tasks.utils.robot.MAX_SPEED = 1.5
swarm_tasks.utils.robot.MAX_ANGULAR: 0.3
np.random.seed(42)
random.seed(42)
CIRCLE_RADIUS = 4

from swarm_tasks.simulation import simulation as sim
from swarm_tasks.simulation import visualizer as viz
	
#Required modules
import swarm_tasks.controllers.base_control as base_control
from swarm_tasks.modules.formations import circle


#Initialize Simulation and GUI 
s = sim.Simulation(num_bots=15, env_name='empty_world', contents_file='attractors')
gui = viz.Gui(s)
gui.show_env()

while(1):
	for b in s.swarm:
		#Base control
		cmd = base_control.base_control(b)
		cmd+= base_control.obstacle_avoidance(b)
		
		#Behaviour
		cmd+=circle(b,CIRCLE_RADIUS)
		
		#Execute
		cmd.exec(b)
		
	gui.update()
