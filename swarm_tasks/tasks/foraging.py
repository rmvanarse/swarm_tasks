from shapely.geometry import Point
from shapely.ops import nearest_points

import swarm_tasks.utils as utils

import swarm_tasks.controllers.base_control as base_control
from swarm_tasks.modules.aggregation import aggr_centroid, aggr_field

from swarm_tasks.modules.surround import surround_attractor
from swarm_tasks.modules.decisions import switch_stoch as switch
from swarm_tasks.modules.formations import line

from swarm_tasks.tasks import area_coverage as cvg



PERIMETER_NEIGHBOURHOOD_RADIUS = 4
LINE_NEIGHBOURHOOD_RADIUS = 3
utils.robot.MAX_ANGULAR = 1

STATE_LINE = 3
STATE_SEARCH = 0
STATE_DEPLOY = 4	#Change to 4

STATE_ENGAGE = 1	#Engage state has to be 1 (default in sim_tests)
STATE_ENDPOINT = 2	#This state has no base control



def gather_resources(bot, use_base_control=True,\
					thresh_dist=0.2):

	num_contact = 0
	item_visible = False
	for item in bot.sim.contents.items:
		if item.subtype in ['resource', 'nest']:
			pos = Point(bot.get_position())
			p1,p2 = nearest_points(pos, item.polygon)
			r = p2.distance(p1)
			
			if r<= bot.size+thresh_dist:
				num_contact += 1
				item_visible = True
				continue
			
			if r < PERIMETER_NEIGHBOURHOOD_RADIUS:
				item_visible = True
				continue

	neighbours_line = bot.neighbours(PERIMETER_NEIGHBOURHOOD_RADIUS,single_state=True, state=STATE_LINE)
	neighbours_deployed = bot.neighbours(PERIMETER_NEIGHBOURHOOD_RADIUS,single_state=True, state=STATE_DEPLOY)
	neighbours_waiting = bot.neighbours(PERIMETER_NEIGHBOURHOOD_RADIUS,single_state=True, state=STATE_ENDPOINT)

	#SEARCH
	if bot.state == STATE_SEARCH:
		cmd = cvg.disp_exp_area_cvg(bot, use_base_control=False, \
									exp_weight_params=[1.0,1.5],\
									disp_weight_params=[2.0, 1.0])

		if (len(neighbours_deployed) or len(neighbours_line) or neighbours_waiting):
			switch(bot, STATE_LINE, 0.01)
		
		if item_visible:
			switch(bot, STATE_DEPLOY, 0.005)

		
	#DEPLOYED
	if bot.state == STATE_DEPLOY:
		cmd = surround_attractor(bot)
		switch(bot, STATE_SEARCH, 0.002)
		
		if num_contact:
			bot.state = STATE_ENDPOINT

		switch(bot, STATE_SEARCH, 0.01*len(bot.neighbours(bot.size*3)))

	#WAITING
	if bot.state == STATE_ENDPOINT:
		cmd = aggr_centroid(bot)*0.001
		
		if num_contact and len(neighbours_line):
			switch(bot, STATE_ENGAGE, 0.1)

		if not num_contact:
			bot.state = STATE_DEPLOY

		if num_contact>1:
			bot.state = STATE_SEARCH

		switch(bot, STATE_SEARCH, 0.01*len(bot.neighbours(bot.size*3)))

	#LINE
	if bot.state == STATE_LINE:
		cmd = surround_attractor(bot)
		cmd += line(bot, LINE_NEIGHBOURHOOD_RADIUS, True, [STATE_LINE, STATE_DEPLOY])
		cmd += aggr_centroid(bot, single_state=True, state=STATE_LINE)*0.2
		
		if item_visible:
			switch(bot, STATE_DEPLOY, 0.005)

		if num_contact:
			bot.state = STATE_SEARCH

		switch(bot, STATE_SEARCH, 0.0025)


	#ENGAGED
	if bot.state == STATE_ENGAGE:
		cmd = aggr_centroid(bot, single_state=True, state=STATE_LINE)*2.5
		if not num_contact:
			bot.state = STATE_DEPLOY
		if num_contact>1:
			bot.state = STATE_SEARCH

		if not len(neighbours_line):
			bot.state = STATE_ENDPOINT

		switch(bot, STATE_ENDPOINT, 0.005)



	#Add base control if needed
	if(use_base_control and  bot.state != STATE_ENDPOINT and bot.state != STATE_ENGAGE):
		cmd += base_control.base_control(bot)*0.5
		cmd += base_control.obstacle_avoidance(bot)*0.3

	return cmd