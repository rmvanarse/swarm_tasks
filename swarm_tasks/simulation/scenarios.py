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

EVENT_LOG = "EVENT_LOG\n"
log_flag = False

def contaminations(sim, prob_new=0.0002, wait_time=20):
	"""
	Used in contamination removal task

	Scenario:
		- Contaminations can spawn at random locations with probability prob_new
		- The contaminations keep growing at a fixed rate (area)
		- Robots can 'absorb' or 'neutralize' the contaminant
		- Each robot has a fixed rate of neutralization (default: 1/3 of growth rate)

	"""
	#Create event log for the first call
	global EVENT_LOG, log_flag
	min_containment_bots = 3
	if not log_flag:
		EVENT_LOG += "SCENARIO: RANDOM GROWING CONTAMIATIONS"
		EVENT_LOG += "\nprob_new: "+str(prob_new)
		EVENT_LOG += "\ncontamination growth rate: "+str(min_containment_bots)
		EVENT_LOG += "\ncontamination removal rate: 1.0 per robot"	#Hardcoded in c.update call
		EVENT_LOG += "\n"
		log_flag = True

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
		EVENT_LOG += ("\nSim time:"+str(sim.time_elapsed)+": New contamination at "+str(pos))

	#Update Contaminated area
	for c in sim.contents.items:
		if c.subtype != 'contamination':
			continue
		c.update(min_containment_bots-sim_tests.num_bots_around_item(sim, c, 1))
		if c.radius < utils.robot.DEFAULT_SIZE+0.05:
			EVENT_LOG += "\nSim time:"+str(sim.time_elapsed) +": Contamination removed at " +str(c.pos)
			sim.contents.items.remove(c)
			print("ITEM REMOVED\nNum items: ", len(sim.contents.items))
			if len(sim.contents.items)==0:
				EVENT_LOG += "\nSim time:"+str(sim.time_elapsed)+": Sim Success. All present contaminations removed."


	sim.has_item_moved = True

	#DRBUG:
	if not sim.time_elapsed % 100:
		#print(EVENT_LOG)
		pass
	return None


def movable_resources(sim, num_resources):

	"""
	Used in foraging and resource gathering tasks

	Scenario:
		- Circular resources of random size are spawned at random locations
		- Optionally, a NEST exists at a fixed location
		- Robots can 'attach' to the resources and steer/ move them
		- Speed of movement depends on the size of the resource and number or robots

	"""
	nest_location, nest_size = (1.5,1.5), 3

	if not len(sim.contents.items):
		#Create events log for first call
		global EVENT_LOG, log_flag
		EVENT_LOG += "SCENARIO: FORAGING/ RESOURCE GATHERING"
		EVENT_LOG += "\nnum_resources: " + str(num_resources)

		#Create Nest
		
		sim.contents.items.append(ex.Nest(nest_location, nest_size))
		EVENT_LOG += "\nNest: "+str(nest_location)+" size="+str(nest_size)
		
		#Create Resources in the env
		for i in range(num_resources):
			pos = np.random.rand(2)*(sim.size)
			radius = np.random.rand()*0.5+0.5	#Currently 0.5<=r<=1
			while not sim.check_free(*pos, radius+utils.robot.DEFAULT_SIZE):
				pos = np.random.rand(2)*(sim.size)
			sim.contents.items.append(ex.Resource(pos, radius))
			EVENT_LOG += "\nResource "+str(i+1)+": "+str(pos)+" size="+str(radius)
		#End for
	#End if

	#Move resources
	num_collected = 0
	collected_new = False
	for r in sim.contents.items:
		#Skip if item is not a resource
		if r.subtype != 'resource':
			continue

		#Avoid other items
		weights_dict = {'bots':0.2, 'obstacles':1, 'borders':1, 'goal':0, 'items':1}
		weights_dict_nest = {'bots':0.2, 'obstacles':0, 'borders':0, 'goal':0, 'items':-1.0}
		movers = sim_tests.bots_picking_item(sim,r)
		#weights_dict['bots']=len(movers)
		cmd = potf.get_field(np.array(r.pos), sim, weights=weights_dict,\
							max_dist=r.radius+utils.robot.DEFAULT_SIZE +0.025,\
							order=3,\
							item_types = ['resource'])
		cmd += potf.get_field(np.array(r.pos), sim, weights=weights_dict_nest,\
							max_dist=r.radius +0.025,\
							item_types = ['nest'])


		cmd.exec(r)
		sim.has_item_moved = True

		#Test if  at nest (for logs)
		bool_collected_before = sim_tests.item_at_nest(sim, r, nest_location, nest_size)
		num_collected += bool_collected_before

		#Move using movers
		dir_movement = 0
		n=len(movers)
		if not n:
			continue

		for m in movers:
			dir_movement+= m.theta/n
		Cmd(speed=n/r.weight, dir_=dir_movement).exec(r)

		#Update lof if successfully collected
		bool_collected_after = sim_tests.item_at_nest(sim, r, nest_location, nest_size)
		
		if bool_collected_after and not bool_collected_before:
			EVENT_LOG += "\nSim time:"+str(sim.time_elapsed)+": New resource collected"
			collected_new = True
			num_collected += 1
		print(bool_collected_after)
		print(num_collected)
	#End for
	if collected_new:
		EVENT_LOG += "\nNum collected: "+str(num_collected)+" of "+str(num_resources)
		if num_collected == num_resources:
			EVENT_LOG += "\nSim time: "+str(sim.time_elapsed)+": Success! Foraging completed"

	#DEBUG:
	if not sim.time_elapsed % 100:
		#print(EVENT_LOG)
		pass
	return None


#TODO: Log update: periodic area coverage %check


def get_event_log():
	return EVENT_LOG