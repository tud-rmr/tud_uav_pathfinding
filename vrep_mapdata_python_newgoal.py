# -*- coding: utf-8 -*-
"""
Created on Thu Oct 22 18:47:03 2015

@author: Jonas
"""
#import from Libaries which are usefull
import vrep#needed for the Connection with the Simulator
import sys
import numpy as np#needed for the arrays and some other mathematical operations
import time
import math
from scipy import interpolate#needed for the interpolation functions
import collections #needed for the queue       
import heapq#needed for the queue


#Definition of SquareGrid, a graph which describes the whole area
class SquareGrid:
    def __init__(self, xmax, ymax, zmax):
        self.xmax = xmax
        self.ymax = ymax
        self.zmax = zmax
    #defines the costs for the way between 2 nodes, in our case the cost is the distance, so the algorythm finds the shortest path
    def cost(self, a, b):
        (x1, y1, z1) = a
        (x2, y2, z2) = b
        return math.sqrt((x1-x2)**2+(y1-y2)**2+(z1-z2)**2)
        #return 1
    #checks if a possible node is inside the moveable area    
    def in_bounds(self, id):
        (x, y, z) = id
        return 0 <= x < self.xmax and 0 <= y < self.ymax and 0 <= z < self.zmax
    #checks if something is in between 2 nodes, so that the object cant move this direction
    def passable(self, id):
        (x,y,z)=id
        #arr[] is an array with the information about the obstacles in the area, filled by sensor information
        if arr[x,y,z]==0:
            boolean=3
        else:
            boolean=2
        return boolean==3
    #returns all possible nodes to move on, means all theoretical possible nodes next to the given node, filtered by in_bounds() and passable()
#    def neighbors(self, id):
#        (x, y, z) = id
#        results = [(x+1, y, z), (x, y-1, z), (x-1, y, z), (x, y+1, z),(x+1,y+1, z),(x+1,y-1, z),(x-1,y-1, z),(x-1,y+1, z),
#                   (x, y, z+1),(x+1, y, z+1), (x, y-1, z+1), (x-1, y, z+1), (x, y+1, z+1),(x+1,y+1, z+1),(x+1,y-1, z+1),(x-1,y-1, z+1),(x-1,y+1, z+1),
#                   (x, y, z-1),(x+1, y, z-1), (x, y-1, z-1), (x-1, y, z-1), (x, y+1, z-1),(x+1,y+1, z-1),(x+1,y-1, z-1),(x-1,y-1, z-1),(x-1,y+1, z-1)]
#        results = filter(self.in_bounds, results)
#        results = filter(self.passable, results)
#        return results
    def neighbors(self, id):
        (x, y, z) = id
        results = [(x+1, y, z), (x, y-1, z), (x-1, y, z), (x, y+1, z),(x, y, z+1),(x, y, z-1)]
        results = filter(self.in_bounds, results)
        results = filter(self.passable, results)
        return results
        

#this queue structure is needed for the A* algorythm and the difference to the Dijkstra algorythm, which would return the same result, but normally needs more time
class PriorityQueue:
    def __init__(self):
        self.elements = []
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self):
        return heapq.heappop(self.elements)[1]
    
    
#this function is the difference between A* and Dijkstra, it returns the distance between a node and the goal, if 2 paths have the same cost it will use the path which is nearer to the goal       
def heuristic(a, b):
    (x1, y1, z1) = a
    (x2, y2, z2) = b
    return math.sqrt((x1-x2)**2+(y1-y2)**2+(z1-z2)**2)

#this is the Pathfinding algorythm A*, implemented by using the defined functions and datastructures
def a_star_search(graph, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    while not frontier.empty():
        current = frontier.get()
        
        if current == goal:
            
            break
        for next in graph.neighbors(current):
            
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current
                
    
    return came_from, cost_so_far


vrep.simxFinish(-1) # just in case, close all opened connections

clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP

##Create the empty Grid, to save obstacle information later
#arr=np.ndarray(shape=(30,30,10),dtype=float)
##Initiate Sensor 1
#errorCode,sensor1=vrep.simxGetObjectHandle(clientID,'Sensor_1',vrep.simx_opmode_oneshot_wait)
#errorCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor (clientID,sensor1,vrep.simx_opmode_streaming)            
##Initiate Sensor 2
#errorCode,sensor2=vrep.simxGetObjectHandle(clientID,'Sensor_2',vrep.simx_opmode_oneshot_wait)
#errorCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor (clientID,sensor2,vrep.simx_opmode_streaming)            
##wait some time, for the end of the sensor initiation
#time.sleep(3)
#                                  
##some for loops to move the sensors throug all the area and detect all obstacles
#index=0
#index2=0
#index3=0
#for index in range(30):#x-Direction
#    for index2 in range(30):#y-Direction
#        for index3 in range(10):#z-Direction
#        #for index3 in range(1):
#             #calculate new Sensor position in m
#            x=0.4+0.4*index#start value + value correspondending to the array index
#            y=0.2+0.4*index2
#            z=0.3+0.4*index3
#            #move Sensor 1 and 2 to the next position in the grid
#            vrep.simxSetObjectPosition (clientID,sensor1,-1,(x,y,z),vrep.simx_opmode_oneshot)
#            vrep.simxSetObjectPosition (clientID,sensor2,-1,(x-0.4,y,z),vrep.simx_opmode_oneshot)
#            #wait till the sensor had time to check for objects
#            time.sleep(0.3)            
#            #read Sensor 1 and 2
#            errorCode,detectionState1,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor (clientID,sensor1,vrep.simx_opmode_buffer)            
#            errorCode,detectionState2,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor (clientID,sensor2,vrep.simx_opmode_buffer)            
#            #save the sensor-results in the "arr" array         
#            if (detectionState1 or detectionState2):            
#                arr[index,index2,index3]=1   #value 1 means obstacle
#            else:
#                arr[index,index2,index3]=0   #vale 0 means nothing in the way

#create a grid
grid=SquareGrid(30,30,10)
#start the Pathfinding algorythm
#find the goal point
errorcode,newgoal_handle=vrep.simxGetObjectHandle(clientID,'goal_new',vrep.simx_opmode_oneshot_wait)
#errorCode,newgoal_position=vrep.simxGetObjectPosition(clientID,newgoal_handle,-1,vrep.simx_opmode_streaming)
errorCode,newgoal_position=vrep.simxGetObjectPosition(clientID,newgoal_handle,-1,vrep.simx_opmode_buffer)

#came_from, cost_so_far = a_star_search(grid, (0, 0, 0), (8,16,0))
came_from, cost_so_far = a_star_search(grid, (0, 0, 0), newgoal_position)

#this function is need to get the path(in points, nodes) from the results of the pathfinding algorythm
def reconstruct_path(came_from, start, goal):
    current = goal
    path = [current]
    while current != start:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path

#use the reconstruct function to get the path
#path2=reconstruct_path(came_from,(0,0,0),(8,16,0))
path2=reconstruct_path(came_from, (0,0,0), newgoal_position)
#need to control the quadrocopter later, by just moving away the target slowly
errorCode,UAV=vrep.simxGetObjectHandle(clientID,'UAV_target',vrep.simx_opmode_oneshot_wait)

#function to check for obstacles between 2 nodes, which are not directly neighbors
def collision(a,b):
    (x1,y1,z1)=a
    (x2,y2,z2)=b
    out=0;
    #straight line between the 2 nodes, 1000 points in between are calculated
    for l in range(1000):
        x=x1+(x2-x1)*(l+1)/1000
        y=y1+(y2-y1)*(l+1)/1000     
        z=z1+(z2-z1)*(l+1)/1000       
        #round the result to get the array indexs 
        x=round(x,0)
        y=round(y,0)
        z=round(z,0)
        out=out+arr[x,y,z]
    #returns only true, if all nodes checked in the array returned the value 0 which means no obstacle
    return out==0

def angle_calculationx(a,b):
    dot = np.dot(a,b)
    x_modulus = np.sqrt(a[0]**2+a[1]**2+a[2]**2)
    y_modulus = np.sqrt(b[0]**2+b[1]**2+b[2]**2)
    cos_angle = dot / x_modulus / y_modulus 
    angle = np.arccos(cos_angle) # Winkel in Bogenmaß 
    ang2=angle*360/2/np.pi
    #print ang2
    if a[1]>0:
        return angle
    else:
        return -angle

def angle_calculationy(a,b):
    dot = np.dot(a,b)
    x_modulus = np.sqrt(a[0]**2+a[1]**2+a[2]**2)
    y_modulus = np.sqrt(b[0]**2+b[1]**2+b[2]**2)
    cos_angle = dot / x_modulus / y_modulus 
    angle = np.arccos(cos_angle) # angle in radiant
    if a[2]>0:
        return angle
    else:
        return -angle

errorCode,Ball=vrep.simxGetObjectHandle(clientID,'A_star_points',vrep.simx_opmode_oneshot_wait)
objectHandles=np.array([Ball])
for next in path2:
    (a,b,c)=next
    x=0.4+0.4*a
    y=0.2+0.4*b
    z=0.3+0.4*c
    returnCode,newObjectHandles=vrep.simxCopyPasteObjects(clientID,objectHandles,vrep.simx_opmode_oneshot_wait)
    Ball_new=newObjectHandles[0]
    vrep.simxSetObjectPosition (clientID,Ball_new,-1,(x,y,z),vrep.simx_opmode_oneshot)
     
#interpolation
#1. step elimination of unnecessary nodes in the path, makes the path shorter, because of more direct movements
in_progress=1
while in_progress>0:
    in_progress=0
    i=0
    while i <(len(path2)-2):
        if collision(path2[i],path2[i+2]):
            path2.pop(i+1)
            in_progress=1
        i=i+1
       
#Mark the points in the simulation 
errorCode,Ball=vrep.simxGetObjectHandle(clientID,'A_star_points_filtered',vrep.simx_opmode_oneshot_wait)
objectHandles=np.array([Ball])
for next in path2:
    (a,b,c)=next
    x=0.4+0.4*a
    y=0.2+0.4*b
    z=0.3+0.4*c
    returnCode,newObjectHandles=vrep.simxCopyPasteObjects(clientID,objectHandles,vrep.simx_opmode_oneshot_wait)
    Ball_new=newObjectHandles[0]
    vrep.simxSetObjectPosition (clientID,Ball_new,-1,(x,y,z),vrep.simx_opmode_oneshot)
       
#2. step interpolate the remainging corner points of the path by using different degrees of polynoms
data=np.ndarray(shape=(len(path2),3),dtype=float)   #create an array of float type for the input points
#fill the array with the Pathdata
for i in range(len(path2)):
    (x,y,z)=path2[i]
    data[i,0]=x
    data[i,1]=y
    data[i,2]=z
#arrange the data to use the function
data = data.transpose()
#interpolate polynom degree 1
tck, u= interpolate.splprep(data,k=1,s=0.1)
linear = interpolate.splev(np.linspace(0,1,100), tck)
#interpolate polynom degree 2
tck, u= interpolate.splprep(data,k=2,s=0.1)
quadratic = interpolate.splev(np.linspace(0,1,100), tck)
#interpolate polynom degree 3
tck, u= interpolate.splprep(data,k=3,s=0.1)
qubic = interpolate.splev(np.linspace(0,1,100), tck)
#Linear
#get the Object handle of the green arrow
errorCode,Arrow=vrep.simxGetObjectHandle(clientID,'Arrow',vrep.simx_opmode_oneshot_wait)
objectHandles=np.array([Arrow])
#put arrows on every point the trajectory pointing in the direction of the next point
for next in range(100):
    a1=linear[0]
    b1=linear[1]
    c1=linear[2]
    a=a1[next]
    b=b1[next]
    c=c1[next]
    if next<99:
        a2=a1[next+1]
        b2=b1[next+1]
        c2=c1[next+1]
    else:
        a2=a
        b2=b
        c2=c
    adiff=a2-a
    bdiff=b2-b
    cdiff=c2-c
    x=0.4+0.4*a
    y=0.2+0.4*b
    z=0.3+0.4*c
    returnCode,newObjectHandles=vrep.simxCopyPasteObjects(clientID,objectHandles,vrep.simx_opmode_oneshot_wait)
    Arro=newObjectHandles[0]
    vrep.simxSetObjectPosition (clientID,Arro,-1,(0,0,0),vrep.simx_opmode_oneshot)
    returnCode=vrep.simxSetObjectOrientation(clientID,Arro,-1,[angle_calculationx([0,bdiff,cdiff],[0,0,-1]),-angle_calculationy([0,bdiff,cdiff],[adiff,bdiff,cdiff]),0],vrep.simx_opmode_oneshot)   
    vrep.simxSetObjectPosition (clientID,Arro,-1,(x,y,z),vrep.simx_opmode_oneshot)
#Quadratic
errorCode,Arrow=vrep.simxGetObjectHandle(clientID,'Arrow2',vrep.simx_opmode_oneshot_wait)
objectHandles=np.array([Arrow])
#put arrows on every point the trajectory pointing in the direction of the next point
for next in range(100):
    a1=quadratic[0]
    b1=quadratic[1]
    c1=quadratic[2]
    a=a1[next]
    b=b1[next]
    c=c1[next]
    if next<99:
        a2=a1[next+1]
        b2=b1[next+1]
        c2=c1[next+1]
    else:
        a2=a
        b2=b
        c2=c
    adiff=a2-a
    bdiff=b2-b
    cdiff=c2-c
    x=0.4+0.4*a
    y=0.2+0.4*b
    z=0.3+0.4*c
    returnCode,newObjectHandles=vrep.simxCopyPasteObjects(clientID,objectHandles,vrep.simx_opmode_oneshot_wait)
    Arro=newObjectHandles[0]
    vrep.simxSetObjectPosition (clientID,Arro,-1,(0,0,0),vrep.simx_opmode_oneshot)
    returnCode=vrep.simxSetObjectOrientation(clientID,Arro,-1,[angle_calculationx([0,bdiff,cdiff],[0,0,-1]),-angle_calculationy([0,bdiff,cdiff],[adiff,bdiff,cdiff]),0],vrep.simx_opmode_oneshot)  
    vrep.simxSetObjectPosition (clientID,Arro,-1,(x,y,z),vrep.simx_opmode_oneshot)
#Qubic
errorCode,Arrow=vrep.simxGetObjectHandle(clientID,'Arrow3',vrep.simx_opmode_oneshot_wait)
objectHandles=np.array([Arrow])
#put arrows on every point the trajectory pointing in the direction of the next point
for next in range(100):
    a1=qubic[0]
    b1=qubic[1]
    c1=qubic[2]
    a=a1[next]
    b=b1[next]
    c=c1[next]
    if next<99:
        a2=a1[next+1]
        b2=b1[next+1]
        c2=c1[next+1]
    else:
        a2=a
        b2=b
        c2=c
    adiff=a2-a
    bdiff=b2-b
    cdiff=c2-c
    x=0.4+0.4*a
    y=0.2+0.4*b
    z=0.3+0.4*c
    returnCode,newObjectHandles=vrep.simxCopyPasteObjects(clientID,objectHandles,vrep.simx_opmode_oneshot_wait)
    Arro=newObjectHandles[0]
    vrep.simxSetObjectPosition (clientID,Arro,-1,(0,0,0),vrep.simx_opmode_oneshot)
    returnCode=vrep.simxSetObjectOrientation(clientID,Arro,-1,[angle_calculationx([0,bdiff,cdiff],[0,0,-1]),-angle_calculationy([0,bdiff,cdiff],[adiff,bdiff,cdiff]),0],vrep.simx_opmode_oneshot) #winkel_berechnen([0,bdiff,cdiff],[adiff,bdiff,cdiff])
    vrep.simxSetObjectPosition (clientID,Arro,-1,(x,y,z),vrep.simx_opmode_oneshot)    
    #commands to move the quadrocopter    
    #vrep.simxSetObjectPosition (clientID,UAV,-1,(x,y,z),vrep.simx_opmode_oneshot)
    #time.sleep(1)
#returnCode,position=vrep.simxGetJointPosition(clientID,UAV,vrep.simx_opmode_buffer)

#returnCode=vrep.simxSetJointTargetPosition(clientID,UAV,1.0,vrep.simx_opmode_oneshot)

print 'finished'
