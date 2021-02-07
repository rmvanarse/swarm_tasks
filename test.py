import swarm_tasks

from swarm_tasks.simulation import simulation as sim
from swarm_tasks.simulation import visualizer as viz

import swarm_tasks.utils as utils
import swarm_tasks.envs as envs
import swarm_tasks.controllers as ctrl

s = sim.Simulation(env_name='rectangles')
gui = viz.Gui(s)

gui.show_env()
gui.show_all()
gui.show_neighbourhood(s.swarm[0])

#gui.animate(s.swarm[0].step, 200, 0.05)
for i in range(100):

	for b in s.swarm:
		#b.move(3.1, 1)
		cmd = ctrl.command.Cmd(speed=1, dir_=3.1)
		cmd.exec(b)
	#s.swarm[0].step()
	#s.swarm[2].step()
	#s.swarm[0].turn(0.05)
	gui.update()


gui.run()


#Print tests
