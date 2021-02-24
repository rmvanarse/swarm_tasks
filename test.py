import swarm_tasks

from swarm_tasks.simulation import simulation as sim
from swarm_tasks.simulation import visualizer as viz

import swarm_tasks.utils as utils
import swarm_tasks.envs as envs
import swarm_tasks.controllers as ctrl
import swarm_tasks.controllers.potential_field as potf
import swarm_tasks.controllers.base_control as base_control

from swarm_tasks.modules.aggregation import aggr_centroid, aggr_field
from swarm_tasks.modules.dispersion import disp_field
from swarm_tasks.modules.formations import circle

import numpy as np

#s = sim.Simulation(env_name='empty_world')
s = sim.Simulation(env_name='rectangles')


gui = viz.Gui(s)

gui.show_env()


gui.show_bots()
gui.show_neighbourhood(s.swarm[0])

s.swarm[0].set_goal(6,1)
grid = np.random.rand(50,50)
#gui.show_grid(np.random.randn(50,50)>0.5)

iter_=0


while 1:
	for b in s.swarm:
		#b.move(3.1, 1)
		#cmd = ctrl.command.Cmd(speed=1, dir_=-2*i*np.pi/50)
		#cmd = ctrl.command.Cmd([2*np.cos(2*i*np.pi/50),2*np.sin(2*i*np.pi/50)])
		#cmd = potf.get_field((b.get_position()),b.sim)
		x,y = b.get_position()
		grid[-int(y*50/s.size[0]), int(x*50/s.size[1])]=1
		
		#cmd = potf.get_field((b.get_position()),b.sim, goal_set=b.goal_exists(), goal=b.goal)
		cmd = base_control.base_control(b)
		cmd+= base_control.obstacle_avoidance(b)
		
		cmd+=circle(b,5)

		#cmd+= disp_field(b)
		#cmd+=aggr_centroid(b)
		#cmd+=aggr_field(b)
		cmd.exec(b)

	#if not iter_%100:
		#gui.show_grid(grid)
	#s.swarm[0].step()
	#s.swarm[2].step()
	#s.swarm[0].turn(0.05)
	gui.update()


gui.run()



#Print tests
