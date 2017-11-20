#3D pathgeneration and pathfollowing for UAV in VREP
This repository contains python code, that can be ran in combination with the Robot Simulation Platform VREP. This set if scripts simulate Pathfinding, Pathsmoothing and Pathfollowing for an UAV in some different scenearios (one small test area and a bigger area showing 2 buildings of our univesity)

## Requirenments:

Ubuntu 16.04
ROS Kinetic
VREP (With compiled ROS plugin, check VREP documentation)
Spyder3 (for comfortable development)

## Installation

1. Install the latest VREP version and compile the ROS Plugin

2. Clone the repository

3. Open VREP (from terminal so the ROS plugin works) and open the scene named "vrep/columns_and_blocks.ttt"

4. Open spyder3

5. In Spyder, run the main.py script which starts the mapdata generation, path generation and pathfollowing algorithms.

## Description of the simulation

In VREP a quadcopter model is implemented with a Velocity Controller for the X and Y direction of movement, and a position controller for the height (Z). The controller is
quite simple and it can be modified as necessary.

The scripts in python use the VREP ROS plugin in order to connect to the simulation
and send/receive information. From VREP we obtain the position of the objects on the scene (for building a map), the current quadcopter position and the goal (represented as red sphere in the simulation). The script uses this information to calculate a path from current UAV position to the goal and sends succesive commands so the UAV follows the path.


The steps of the script are as follows:

	Step 1:

	Generates an 3D array with the occupancy grid for the current scene in VREP. This is being implemented in the mapgen script. VREP doesn't provide a good way for map generation, so we use a brute force approach. We create an array of proximity sensors on a plane an VREP, then we move the sensors around the environment detecting the objects in order to fill the occupancy grid. This array is stored in a text file for later use, if the scene is the same the mapgen script uses this text file instead of generating the map.

	Step 2:

	The current UAV position and the goal position is obtained from VREP.

	Step 3:

	Using the generated map, the currrent position and the goal position, a path is generated using the A* algorithm. This discrete path is then smoothed using polinomials.

	Step 4:

	A pathfollowing algorithm is implemented based on a velicity vector field approach. We define an aproximation vector and a tangential vector to the closest point of the path from the current UAV position. These two vectors are then fused together for the final velocity reference of the UAV which is then sent to VREP.
