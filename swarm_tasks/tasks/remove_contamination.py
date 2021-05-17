from shapely.geometry import Point
from shapely.ops import nearest_points

import swarm_tasks.utils as utils

import swarm_tasks.controllers.base_control as base_control
from swarm_tasks.modules.aggregation import aggr_centroid
from swarm_tasks.modules.follow import follow_leader

from swarm_tasks.modules.surround import surround_attractor
from swarm_tasks.tasks import area_coverage as cvg


PERIMETER_NEIGHBOURHOOD_RADIUS = 4

STATE_SEARCH = 0
STATE_FOLLOW = 1
STATE_PERIMETER = 2
STATE_RUSH = 3

def remove_contamination(bot, use_base_control=True,\
						surround_weight = 2.5,\
						search_weight = 2.0,\
						thresh_dist = 0.25,\
						use_follow_state=True):

	#If item is in contact or item is seen, change state
	flag=False
	for item in bot.sim.contents.items:
		if item.subtype == 'contamination':
			pos = Point(bot.get_position())
			p1,p2 = nearest_points(pos, item.polygon)
			r = p2.distance(p1)
			
			if r<= bot.size+thresh_dist:
				flag=True
				bot.set_state(STATE_PERIMETER)
				break
			
			if r < PERIMETER_NEIGHBOURHOOD_RADIUS:
				flag=True
				bot.set_state(STATE_RUSH)
				continue #continue searching for STATE_PERIM
	
	if not flag:
		bot.set_state(STATE_SEARCH)

	#Task

	if bot.state ==STATE_SEARCH:
		cmd = cvg.disp_exp_area_cvg(bot, use_base_control=False, \
									exp_weight_params=[1.0,1.5],\
									disp_weight_params=[2.0, 1.0]) * search_weight

		#If a RUSH bot is seen, follow it
		#STATE_FOLLOW are only the robots that see a rushing robot
		#If a STATE_FOLLOW neighbour is seen, it is followed without changing state
		neighbours_rush = bot.neighbours(PERIMETER_NEIGHBOURHOOD_RADIUS,single_state=True, state=STATE_RUSH)
		neighbours_follow = bot.neighbours(PERIMETER_NEIGHBOURHOOD_RADIUS,single_state=True, state=STATE_FOLLOW)
		
		if len(neighbours_rush)>0:
			cmd+= follow_leader(bot, neighbours_rush[0])*3
			if use_follow_state:
				bot.set_state(STATE_FOLLOW) 
		
		elif len(neighbours_follow)>0:
			cmd+= follow_leader(bot, neighbours_follow[0])*3

	if bot.state==STATE_RUSH:
		cmd =surround_attractor(bot) #Goes slow in hopes of being seen
		cmd += base_control.base_control(bot)	#Extra obst. avoidance (tuning)

	if bot.state ==STATE_PERIMETER:
		cmd =surround_attractor(bot)*surround_weight
		cmd += cvg.disp_exp_area_cvg(bot, use_base_control=False, exp_weight_params=[0.5,1])

	#Add base control if needed
	if(use_base_control):
		cmd += base_control.base_control(bot)*0.5
		#ADD FIELD WEIGHT FOR ITEMS: REDUCE MIN DIST,INCREASE FIELD
		field_weights={'bots':1.0, 'obstacles':0.5, 'borders':0.5, 'goal':-3, 'items':0.25}
		cmd += base_control.obstacle_avoidance(bot, field_weights)


	return cmd
