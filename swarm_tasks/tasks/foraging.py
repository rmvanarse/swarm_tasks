from shapely.geometry import Point
from shapely.ops import nearest_points
import numpy as np

import swarm_tasks.utils as utils

import swarm_tasks.controllers.base_control as base_control
from swarm_tasks.modules.aggregation import aggr_centroid, aggr_field
from swarm_tasks.modules.dispersion import disp_field as disp
from swarm_tasks.modules.surround import surround_attractor
from swarm_tasks.modules.decisions import switch_stoch as switch
from swarm_tasks.modules.formations import line
from swarm_tasks.modules import exploration as exp
from swarm_tasks.modules import follow

from swarm_tasks.tasks import area_coverage as cvg

"""
ToDo:
Load all parameters from logs/params yaml
"""


PERIMETER_NEIGHBOURHOOD_RADIUS = 4.5
LINE_NEIGHBOURHOOD_RADIUS = 1.5
utils.robot.MAX_ANGULAR = 1

STATE_LINE = 3
STATE_SEARCH = 0
STATE_DEPLOY = 4	#Change to 4

STATE_ENGAGE = 1	#Engage state has to be 1 (default in sim_tests)
STATE_ENDPOINT = 2	#This state has no base control



def gather_resources(bot, use_base_control=True,\
					thresh_dist=0.1):

	num_contact = 0
	item_visible = False
	nest_visible = False
	nest_pt = None
	nest_contact=False

	for item in bot.sim.contents.items:
		if item.subtype in ['resource', 'nest']:
			pos = Point(bot.get_position())
			p1,p2 = nearest_points(pos, item.polygon)
			r = p2.distance(p1)

			
			if r<= bot.size+thresh_dist:
				if item.subtype == 'nest':
					nest_pt = np.array(p2)
					nest_visible = True
					nest_contact = True
					bot.state = STATE_ENDPOINT
				num_contact += 1
				item_visible = True
				continue
			
			if r < PERIMETER_NEIGHBOURHOOD_RADIUS:
				if item.subtype == 'nest':
					nest_pt = np.array(p2)
					nest_visible = True
				item_visible = True
				continue

	neighbours_line = bot.neighbours(PERIMETER_NEIGHBOURHOOD_RADIUS,single_state=True, state=STATE_LINE)
	neighbours_deployed = bot.neighbours(PERIMETER_NEIGHBOURHOOD_RADIUS,single_state=True, state=STATE_DEPLOY)
	neighbours_waiting = bot.neighbours(PERIMETER_NEIGHBOURHOOD_RADIUS,single_state=True, state=STATE_ENDPOINT)

	#SEARCH
	if bot.state == STATE_SEARCH:
		# cmd = cvg.disp_exp_area_cvg(bot, use_base_control=False, \
		# 							exp_weight_params=[1.0,1.5],\
		# 							disp_weight_params=[2.0, 1.0])
		cmd = disp(bot, item_types=['all'])*2
		cmd += exp.explore(bot)*1.5

		if nest_contact:
			bot.unstuck(bot.size)
			print("UNSTUCK!!")

		if (len(neighbours_line) or neighbours_waiting):
			switch(bot, STATE_LINE, 0.1)
			pass

		if item_visible:
			switch(bot, STATE_DEPLOY, 0.005)

		
	#DEPLOYED
	if bot.state == STATE_DEPLOY:
		cmd = surround_attractor(bot)
		switch(bot, STATE_SEARCH, 0.002)
		
		if num_contact:
			#bot.state = STATE_ENDPOINT	#
			if nest_visible:
				cmd = follow.follow_point(bot, nest_pt)*0.01
			else:
				cmd = aggr_centroid(bot, single_state=True, state=STATE_LINE)*0.01
			
			if (len(neighbours_line) or nest_visible):
				switch(bot, STATE_ENGAGE,\
						0.3*nest_visible + 0.3*(len(neighbours_line)>0)*(not nest_visible))
			switch(bot, STATE_ENGAGE, 0.05)
		
		switch(bot, STATE_SEARCH, 0.01*len(bot.neighbours(bot.size*3)))

	#HOME
	if bot.state == STATE_ENDPOINT:
		cmd = surround_attractor(bot)

		if not num_contact:
			bot.state = STATE_DEPLOY

		if num_contact>1:
			bot.state = STATE_SEARCH

		switch(bot, STATE_SEARCH, 0.02*len(neighbours_waiting))

	#LINE
	if bot.state == STATE_LINE:
		cmd = line(bot, LINE_NEIGHBOURHOOD_RADIUS, True, [STATE_LINE, STATE_ENDPOINT, STATE_DEPLOY])*2.5
		#cmd = surround_attractor(bot)*0.2
		#cmd += aggr_centroid(bot, single_state=True, state=STATE_LINE)*0.2
		#cmd += exp.explore(bot)
		cmd += base_control.base_control(bot)
		
		if item_visible:
			switch(bot, STATE_DEPLOY, 0.003*(not nest_visible)+0.001)
		
		if not nest_visible:
			cmd += exp.explore(bot)*0.75 #(Replace with linearly increasing truncated wt)
			cmd += aggr_centroid(bot)*0.1
			cmd += surround_attractor(bot)*0.05
		else:
			cmd += disp(bot)*0.2

		if num_contact:
			bot.state = STATE_SEARCH
		if not len(neighbours_line) + len(neighbours_waiting):
			switch(bot, STATE_SEARCH, 0.075)	#Tune

		switch(bot, STATE_SEARCH, 0.0025)


	#ENGAGED
	if bot.state == STATE_ENGAGE:
		if nest_visible:
			cmd = follow.follow_point(bot, nest_pt)
		else:
			cmd = aggr_centroid(bot, single_state=True, state=STATE_LINE)
	

		if not num_contact:
			bot.state = STATE_DEPLOY
		if num_contact>1:
			bot.state = STATE_SEARCH

		if not len(neighbours_line):
			switch(bot, STATE_DEPLOY, 0.2)

		switch(bot, STATE_DEPLOY, 0.005)



	#Add base control if needed -- IS ALWAYS NEEDED!!
	if(use_base_control and bot.state != STATE_ENGAGE):
		cmd += base_control.base_control(bot)*0.5
		field_weights={'bots':1, 'obstacles':1, 'borders':2, 'goal':-3, 'items':1}
		if bot.state == STATE_DEPLOY:
			field_weights['items']=0.00
		elif bot.state == STATE_ENDPOINT:
			field_weights['bots']=3.0
			field_weights['items']=0.00
		cmd += base_control.obstacle_avoidance(bot, field_weights)*0.3

	return cmd
