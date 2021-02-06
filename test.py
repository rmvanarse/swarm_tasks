import swarm_tasks

from swarm_tasks.simulation import simulation as sim
from swarm_tasks.simulation import visualizer as viz

import swarm_tasks.utils as utils
import swarm_tasks.envs as envs

s = sim.Simulation()
gui = viz.Gui(s)

gui.show_all()
gui.show_neighbourhood(s.swarm[0])

#gui.animate(s.swarm[0].step, 200, 0.05)
for i in range(20):

	s.swarm[0].step()
	#s.swarm[0].turn(0.05)
	gui.update()

gui.run()


#Print tests
env = envs.world.World(filename='swarm_tasks/envs/worlds/empty_world.yaml')