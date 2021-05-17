import sys, yaml, os, time
import swarm_tasks
import swarm_tasks.utils.robot as robot

"""
Automatically creating and saving logs
"""

def log_robot():
	params = str()
	params += "# ROBOT PARAMETERS\n"
	params += "\nDEFAULT_NEIGHBOURHOOD_VAL: "+str(robot.DEFAULT_NEIGHBOURHOOD_VAL)
	params += "\nDEFAULT_SIZE: " + str(robot.DEFAULT_SIZE)
	params += "\nMAX_SPEED: " + str(robot.MAX_SPEED) 
	params += "\nMAX_ANGULAR: " + str(robot.MAX_ANGULAR) 
	params += "\nDEFAULT_STATE: " + str(robot.DEFAULT_STATE)
	params += "\nSEED: " + str(swarm_tasks.SEED)+" #For random & np.random"

	return params+"\n\n"

save_path = None


def save_log(logs, path, name, ext=".yaml"):
	#Log list is an array
	#Also join time to name
	global save_path
	if save_path == None:
		time_ = time.strftime("%Y-%m-%d__%H-%M-%S")
		filename = name+time_+ext
		save_path = os.path.join(path, filename)

	full_log = str()
	for l in logs:
		full_log += l +'\n'
	f = open(save_path, "w")
	f.write(full_log)
	f.close()
	return full_log

def print_params(log):
	print("Simulation parameters at "+time.strftime("%Y-%m-%d__%H-%M-%S")+'\n\n')
	print(log)