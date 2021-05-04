import sys, yaml, os, time

import swarm_tasks.utils.robot as robot

def log_robot():
	params = str()
	params += "# ROBOT PARAMETERS\n"
	params += "\nDEFAULT_NEIGHBOURHOOD_VAL: "+str(robot.DEFAULT_NEIGHBOURHOOD_VAL)
	params += "\nDEFAULT_SIZE: " + str(robot.DEFAULT_SIZE)
	params += "\nMAX_SPEED: " + str(robot.MAX_SPEED) 
	params += "\nMAX_ANGULAR: " + str(robot.MAX_ANGULAR) 
	params += "\nDEFAULT_STATE: " + str(robot.DEFAULT_STATE)

	return params+"\n\n"

def save_log(logs, path, name):
	#Log list is an array
	#Also join time to name
	time = time.strftime("%Y-%m-%d__%H-%M-%S")
	pass

def print_params(log):
	print("Simulation parameters at "+time.strftime("%Y-%m-%d__%H-%M-%S")+'\n\n')
	print(log)