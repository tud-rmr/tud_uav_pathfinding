# -*- coding: utf-8 -*-
"""
Created on Sun Nov 01 15:34:40 2015

@author: Jonas, Raul Acuna
"""

#UAV_main.py
from vrep_interface import vrep
from map_generation import mapgen
from path_finding import pathfinding
from vrep_interface import uav_vrep
import numpy as np
import time
from scipy import ndimage

#start Connection to V-REP
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP

#data=[0,0,1,0,0,0]
#packedData=vrep.simxPackFloats(data)

vrep.simxClearStringSignal(clientID,'Command_Twist_Quad',vrep.simx_opmode_oneshot)
#vrep.simxSetStringSignal(clientID,'Command_Twist_Quad',packedData,vrep.simx_opmode_oneshot)

#generate mapdata, load data if mapdata for scene exist
mapdata=mapgen.mapgen_fast("columns_and_blocks",16,16,10,clientID)

# Extending obstacles boundaries using dilation
mapdata = ndimage.binary_dilation(mapdata).astype(np.int64)

# Get start and goal data from v-REP
start_position=uav_vrep.getPosition(clientID,'UAV_target')
goal_position=uav_vrep.getPosition(clientID,'goal_new')
print ("Start position:", start_position)
print ("Goal position:", goal_position)

# Pathfinding
print ("start pathfinding")
start_time = time.time()
path=pathfinding.search(goal_position,start_position,"astar",3,mapdata)
print("Duration of pathfinding", (time.time() - start_time))
#path is a Python list wich contains 3 arrays. For example:
#x = np.array([1,2,3,4,5])
#y = np.array([1,2,3,4,5])
#z = np.array([1,2,3,4,5])
#path = [x,y,z]
#path=UAV_pathfinding.interpolation_polynom(path,3)

#function to start the signals to transport the data to V-REP(LUA) and give the signal to the UAV-script, that the path is ready
uav_vrep.show_path(path,clientID)

#function to follow the path, generates the needed velocities and heights, that are needed for the LUA-script and streams the needed signals
uav_vrep.follow_path(clientID,path,goal_position)
