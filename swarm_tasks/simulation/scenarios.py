"""
A scenario function can be passed as an argument to run_actions() in Simulation class
"""

import swarm_tasks.utils as utils
#import swarm_tasks.simulation as sim
from swarm_tasks.simulation import simulation, sim_tests
import swarm_tasks.utils.item as ex

import numpy as np


def contaminations(sim, prob_new=0.0001):
	#Rand for new contamination
	rand_ = np.random.rand()

	#Create new contamination
	if ((not len(sim.contents.items) )and sim.time_elapsed<10) or rand_ < prob_new:
		pos = np.random.rand(2)*sim.size
		while not sim.check_free(*pos, 1.0):
			pos = np.random.rand(2)*sim.size

		sim.contents.items.append(ex.Contamination(pos, 0.5))

	#Update Contaminated area
	for c in sim.contents.items:
		if c.subtype != 'contamination':
			continue
		c.update(3-sim_tests.num_bots_around_item(sim, c, 1))
		if c.radius < 0.4:
			sim.contents.items.remove(c)
			print("ITEM REMOVED\nNum items: ", len(sim.contents.items))

	sim.has_item_moved = True

	return None