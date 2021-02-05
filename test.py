import swarm_tasks

from swarm_tasks.simulation import simulation as sim
from swarm_tasks.simulation import visualizer as viz

import swarm_tasks.utils as utils

s = sim.Simulation()
gui = viz.Gui(s)

gui.show_all()