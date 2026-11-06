import swarm_tasks.utils as utils

import swarm_tasks.controllers.base_control as base_control
from swarm_tasks.modules.dispersion import disp_field
from swarm_tasks.modules import exploration as exp
from swarm_tasks.modules.formations import line,circle
from swarm_tasks.utils.weight_functions import linear_trunc, hyperbolic,sinusoidal,triangular


NEIGHBOURHOOD_RADIUS = 4.5


def disp_exp_area_cvg(bot, use_base_control=True,\
					disp_weight_function = hyperbolic,\
					exp_weight_function = hyperbolic,\
					disp_weight_params = [],\
					exp_weight_params = []):

	""""
	Weighted combination of dispersion and random motion
	Begins with a low weight for random motion and high for dispersion
	Weights change as per their weight functions
	Ends with high random motion and low dispersion
	"""

	"""
	Args:
		<task>_weight_function: decides how task weights vary with iteration_
		<task>_weight_params are unpacked and passed to the weight function
	Returns: Cmd
	"""
	if bot.sim == None:
		print("ERROR: Bot is not linked to a simulation")
		return 

	t = bot.sim.time_elapsed

	#Calculate weights and create command
	weight_disp = disp_weight_function(t, *disp_weight_params)
	weight_exp = exp_weight_function(t, *exp_weight_params)
	cmd = disp_field(bot, neighbourhood_radius = NEIGHBOURHOOD_RADIUS)*weight_disp
	cmd += exp.explore(bot)*weight_exp
	#cmd += exp.explore(bot)*1.5

	#Add base control if needed (Reccomended)
	if(use_base_control):
		cmd += base_control.exp_control(bot)
		obstacle_field_weights={'bots':1, 'obstacles':1, 'borders':0.3, 'goal':0.0, 'items':0.1}
		cmd += base_control.exp_obstacle_avoidance(bot,obstacle_field_weights)
	return cmd


def goal_area_cvg(i,bot,goal, use_base_control=True,\
					disp_weight_function = hyperbolic,\
					exp_weight_function = linear_trunc,\
					disp_weight_params = [],\
					exp_weight_params = []):

	""""
	Weighted combination of dispersion and random motion
	Begins with a low weight for random motion and high for dispersion
	Weights change as per their weight functions
	Ends with high random motion and low dispersion
	"""

	"""
	Args:
		<task>_weight_function: decides how task weights vary with iteration_
		<task>_weight_params are unpacked and passed to the weight function
	Returns: Cmd
	"""
	if bot.sim == None:
		print("ERROR: Bot is not linked to a simulation")
		return 

	t = bot.sim.time_elapsed

	
	#Calculate weights and create command
	weight_disp = disp_weight_function(t, *disp_weight_params)
	weight_exp = exp_weight_function(t, *exp_weight_params)

	cmd = disp_field(bot, neighbourhood_radius = NEIGHBOURHOOD_RADIUS)*weight_disp
	#cmd += exp.explore(bot)*weight_exp
#	cmd=line(bot)
	#cmd=line(bot, LINE_NEIGHBOURHOOD_RADIUS, True, [STATE_LINE, STATE_ENDPOINT, STATE_DEPLOY])*2.5
	cmd+=circle(bot,100)
	#Add base control if needed (Reccomended)
	if(True):
		cmd += base_control.base_control(i,bot,goal)
		obstacle_field_weights={'bots':1, 'obstacles':3, 'borders':0.3, 'goal':-200.0, 'items':0.1}
		cmd += base_control.obstacle_avoidance(i,bot,goal)
		#cmd+=line(bot)
	
	return cmd

