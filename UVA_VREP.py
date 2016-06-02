# -*- coding: utf-8 -*-
"""
Created on Tue Nov  3 15:31:37 2015

@author: lijinke
"""
import vrep#needed for the Connection with the Simulator
import sys
import numpy as np#needed for the arrays and some other mathematical operations
import time
import math
from scipy import interpolate#needed for the interpolation functions
import collections #needed for the queue       
import heapq#needed for the queue
#vrep.simxFinish(-1) # just in case, close all opened connections

#vrep.simxFinish(-1)
#clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP


def getPosition(clientID,goal_new):
    errorcode,newgoal_handle=vrep.simxGetObjectHandle(clientID,'goal_new',vrep.simx_opmode_oneshot_wait)
    time.sleep(5) 
    #errorCode,newgoal_position=vrep.simxGetObjectPosition(clientID,newgoal_handle,-1,vrep.simx_opmode_streaming)
    errorCode,newgoal_position=vrep.simxGetObjectPosition(clientID,newgoal_handle,-1,vrep.simx_opmode_streaming)
    time.sleep(5) 
    errorCode,newgoal_position=vrep.simxGetObjectPosition(clientID,newgoal_handle,-1,vrep.simx_opmode_buffer)
    return newgoal_position
    
