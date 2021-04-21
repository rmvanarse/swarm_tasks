"""
A scenario function can be passed as an argument to run_actions() in Simulation class
"""

import swarm_tasks.utils as utils
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

	#Create randomly scattered resoruces (assume no other items exist):
	if not len(sim.contents.items):
		for i in range(num_resources):
			pos = np.random.rand(2)*(sim.size)
			radius = np.random.rand()*0.5+0.5	#Currently 0.5<=r<=1
			while not sim.check_free(*pos, radius+utils.robot.DEFAULT_SIZE):
				pos = np.random.rand(2)*(sim.size)
			sim.contents.items.append(ex.Resource(pos, radius))
		#End for
	#End if

	#Move resource with engaged state bots to the centroid of the bots
	for r in sim.contents.items:
		#Skip if item is not a resource
		if r.subtype != 'resource':
			continue
		
		move_flag = False

		#Avoid other items:
		pos = np.array(r.pos)
		new_pos = pos.copy()
		for r2 in sim.contents.items:
			if r==r2:
				continue
			d = pos - np.array(r2.pos)
			if np.linalg.norm(d)<(r.radius+r2.radius+0.1):
				new_pos-=0.1*r.weight**d/(np.linalg.norm(d)+0.00001)
				move_flag = True


		#Get movers
		bots_moving_item = sim_tests.bots_picking_item(sim,r)
		num_bots = len(bots_moving_item)
		if num_bots >= r.min_bots_needed:
			#continue
	
			#Get target direction
			centroid = np.array([0.0,0.0])
			for b in bots_moving_item:
				p = np.array(b.get_position())
				centroid+=p
			centroid/=num_bots
			
			dir_ = centroid-pos
			
			new_pos +=0.1*num_bots*dir_/r.weight
			move_flag = True

		#Go to  new position
			
		if move_flag:
			if sim.check_free(*new_pos, r.radius, ignore=r):

				sim.has_item_moved = True
				r.move_to((new_pos[0], new_pos[1]))

	#TODO: How to ensure resources don't merge?
	return None