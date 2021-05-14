print("Running foraging example 1\nUsing source files for package imports\nPARAMETERS:")
import sys,os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__),'../../..')))
print(sys.path)

import swarm_tasks

from swarm_tasks.simulation import simulation as sim
from swarm_tasks.simulation import visualizer as viz
	
#Required modules
import swarm_tasks.controllers.base_control as base_control
from swarm_tasks.modules.aggregation import aggr_centroid


#Initialize Simulation and GUI 
s = sim.Simulation(env_name='rectangles')
gui = viz.Gui(s)
gui.show_env()

while(1):
	for b in s.swarm:
		#Base control
		cmd = base_control.base_control(b)
		cmd+= base_control.obstacle_avoidance(b)
		
		#Behaviour
		cmd+=aggr_centroid(b)
		
		#Execute
		cmd.exec(b)
		
	gui.update()
