import swarm_tasks

from swarm_tasks.simulation import simulation as sim
from swarm_tasks.simulation import visualizer as viz

import swarm_tasks.utils as utils

s = sim.Simulation()
gui = viz.Gui(s)

gui.show_all()
gui.show_neighbourhood(s.swarm[0])

#gui.animate(s.swarm[0].step, 200, 0.05)
for i in range(100):

	s.swarm[0].step()
	gui.update()

gui.run()
#Print tests
