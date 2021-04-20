import swarm_tasks.utils as utils

import swarm_tasks.controllers.base_control as base_control
from swarm_tasks.modules.dispersion import disp_field
from swarm_tasks.modules import exploration as exp

from swarm_tasks.utils.weight_functions import linear_trunc, hyperbolic


def disp_exp_area_cvg(bot, use_base_control=True,\
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
	
	cmd = disp_field(bot)*weight_disp + exp.explore(bot)*weight_exp
	

	#Add base control if needed
	if(use_base_control):
		cmd += base_control.base_control(bot)
		cmd += base_control.obstacle_avoidance(bot)
	
	return cmd


