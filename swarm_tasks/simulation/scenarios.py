"""
A scenario function can be passed as an argument to run_actions() in Simulation class
"""

import swarm_tasks.utils as utils
from swarm_tasks.controllers import potential_field as potf
from swarm_tasks.controllers.command import Cmd
#import swarm_tasks.simulation as sim
from swarm_tasks.simulation import simulation, sim_tests
import swarm_tasks.utils.item as ex

import numpy as np


def contaminations(sim, prob_new=0.0002, wait_time=20):
	#Rand for new contamination
	rand_ = np.random.rand()

	#Create new contamination
	if sim.time_elapsed < wait_time:
		return None

	if ((not len(sim.contents.items) )and sim.time_elapsed<wait_time+20) or rand_ < prob_new:
		pos = np.random.rand(2)*(sim.size)
		while not sim.check_free(*pos, utils.robot.DEFAULT_SIZE):
			pos = np.random.rand(2)*sim.size

		sim.contents.items.append(ex.Contamination(pos, 0.25))

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


def movable_resources(sim, num_resources):
	if not len(sim.contents.items):
		sim.contents.items.append(ex.Nest((1.5,1.5), 3))
		for i in range(num_resources):
			pos = np.random.rand(2)*(sim.size)
			radius = np.random.rand()*0.5+0.5	#Currently 0.5<=r<=1
			while not sim.check_free(*pos, radius+utils.robot.DEFAULT_SIZE):
				pos = np.random.rand(2)*(sim.size)
			sim.contents.items.append(ex.Resource(pos, radius))
		#End for
	#End if

	#Move resources

	for r in sim.contents.items:
		#Skip if item is not a resource
		if r.subtype != 'resource':
			continue

		#Avoid other items
		weights_dict = {'bots':0.2, 'obstacles':1, 'borders':1, 'goal':0, 'items':1}
		weights_dict_nest = {'bots':0, 'obstacles':0, 'borders':0, 'goal':0, 'items':-1.0}
		movers = sim_tests.bots_picking_item(sim,r)
		#weights_dict['bots']=len(movers)
		cmd = potf.get_field(np.array(r.pos), sim, weights=weights_dict,\
							max_dist=r.radius+utils.robot.DEFAULT_SIZE +0.025,\
							item_types = ['resource'])
		cmd += potf.get_field(np.array(r.pos), sim, weights=weights_dict_nest,\
							max_dist=r.radius +0.025,\
							item_types = ['nest'])


		cmd.exec(r)
		sim.has_item_moved = True

		#Move using movers
		dir_movement = 0
		n=len(movers)
		if not n:
			continue

		for m in movers:
			dir_movement+= m.theta/n

		Cmd(speed=n/r.weight, dir_=dir_movement).exec(r)

	#End for

	return None


