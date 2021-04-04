# Swarmbots - Tasks


The project aims to automate a decentralized swarm of autonomous mobile robots to carry out different tasks, with a focus on motion planning.

The system is completelly decentralized. Each robot only knows its pose and the position of the robots within a speciific neighbourhood radius. The bots do not communicate with each other, and perform tasks through 'collective decision making' algorithms.

Complex swarm applications can be divided into the following basc tasks:

- Aggregation
- Dispersion
- Exploration
- Line formation
- Shape Formation
- Surrounding an item
- Flocking 
- Forraging
- Collective descision making

**Version 0.0.1**
_(In progress...)_
## Structure of the Repository
The repository is astructured as Python3 package. The subfolder ```swarm_tasks``` contains the following modules:

1. **Utilities (```utils```):** This module contains the class definitions and helper functions used in the rest of the repository. The most important of these is the ```Robot``` class. Each robot in th swarm is an instance of this class.

1. **Simulator (```simulation```):** The file ```simulation.py``` defines a simulation class that spawns the environment, obstacles, items and the robos, and simulates their actions and interactions. The ```visualizer.py``` contains a class to visualize this simulation (and other robot properties).
1. **Environments (```envs```):** Contains yaml files defiing the the ```worlds``` (Boundaries, obstacles, etc.) and the ```items``` (objects in the environment with special properties).

1. **Controllers (```controllers```):** The ```command.py``` file defines the ```Cmd``` class, which contains the simplest form of a command that a robot can execute, i.e. a velocity vector. Multiple commands can be added, subtracted and multiplied by a constant, just like vectors, to produce the resutant commanded direction and speed of the robot. This leads to easy stacking up of simultaneous 'behaviors' for each robot.
Each controller implemented in this module returns an instance of ```Cmd```. The potential field motion planner has been implemented, which can take weights for different components of the environment as paramters. Using the potential field method, a base controller has been implemented, which executes basic inter-robot and obstacle avoidance behaviour. 

1. **Modules (```modules```):** All basic tasks are implemented as functions that take the robot object as a parameter, and use the neighbour-positions to return a ```Cmd``` instance. The following behaviours have been implemented
    1. Aggregation: The robots cluster together 
    1. Dispersion: The robots spread out
    1. Exploration: The robots move around randomly
    1. Line formation: The robots form a straight line
    1. Chain formation: The robots form a chain
    1. Circle formation
    1. Surrounding: The robots surround an item

Multiple behaviours can be combined by simply adding the outputs of their respective function. Such combinations can be used to implement complex realworld tasks.
 
## Installation

Clone the package using the following command:

	git clone https://github.com/rmvanarse/swarm_tasks
Install the package from the root directory as follows:

	cd swarm_tasks
	python3 setup.py install


### Usage

Import the package as follows:

	import swarm_tasks

Submodules and functions can be imported similar to standard python packages. For example:

	import swarm_tasks.simulation
	from swarm_tasks.utils import robot

The following are steps to initialize a basic swarm simulation (eg: Circle formation with obstacle avoidance):

	import swarm_tasks

	from swarm_tasks.simulation import simulation as sim
	from swarm_tasks.simulation import visualizer as viz
	
	#Import required modules
	import swarm_tasks.controllers.base_control as base_control
	from swarm_tasks.modules.formations import circle
	
	
	#Initialize Simulation and GUI 
	s = sim.Simulation(env_name='rectangles')
	gui = viz.Gui(s)
	gui.show_env()
	
	while(1):
		for b in s.swarm:
			#Basic behaviours
			cmd = base_control.base_control(b)
			cmd+= base_control.obstacle_avoidance(b)
			
			#Additional behaviours
			cmd+=circle(b,5)
			
			#Execute
			cmd.exec(b)
			
		gui.update()
	


### Authors:
**Rishikesh Vanarse** ( [GitHub](https://github.com/rmvanarse), [Website](https://rmvanarse.github.io) ) 


