# -*- coding: utf-8 -*-
"""
Created on Fri Oct 30 00:17:18 2015

@author: renchen
"""

#import from Libaries which are usefull
from __future__ import division
import vrep#needed for the Connection with the Simulator
import sys
import numpy as np#needed for the arrays and some other mathematical operations
import time
import math
from scipy import interpolate#needed for the interpolation functions
import collections #needed for the queue       
import heapq#needed for the queue
import random
from copy import deepcopy

def search(goal,start,search_type,interpolation,mapdata):
    global mapdata2
    mapdata2=mapdata
    goal2=m_to_grid(goal)
    start2=m_to_grid(start)
    (x,y,z)=mapdata.shape
    #print x,y,z
    grid=SquareGrid(x,y,z)
    if search_type=="astar":
        came_from, cost_so_far = a_star_search(grid, start2, goal2, mapdata) 
        #print goal2, start2
        path=reconstruct_path(came_from,start2,goal2)
    if search_type=="rrt":
        path = rrt_search(grid, start2, goal2, mapdata) 
        #print goal2, start2
        #print path
    #print path
    path=interpolation_skip_points(path)
    #print path
    path=path_grid_to_m(path,start,goal)
    
    
    
    path=interpolation_polynom(path,interpolation)
    #print path
    #print path
    return path
    #return
    
def m_to_grid(point):   
    xm=point[0]
    ym=point[1]
    zm=point[2]
    xgrid=int(round(xm/0.4-1,0))
    ygrid=int(round(ym/0.4-1,0))
    zgrid=int(round(zm/0.4-1,0))
    point=(xgrid,ygrid,zgrid)
    return point
#interpolation
#1. step elimination of unnecessary nodes in the path, makes the path shorter, because of more direct movements       
def interpolation_skip_points(path):
    in_progress=1
    while in_progress>0:
        in_progress=0
        i=0
        if len(path)>2:
            while i <(len(path)-2):
                if collision(path[i],path[i+2]):
                    #if distance(path[i],path[i+2])<4:
                    path.pop(i+1)
                    in_progress=1   
                i=i+1
    print path
    path2=deepcopy(path)  
    n=0
    count_points=0
    while n < (len(path2)-1):
        if distance(path2[n],path2[n+1])>1:
            dis=distance(path2[n],path2[n+1])
            anzahl=int(round(dis,0)-1)
            (x1,y1,z1)=path2[n]
            (x2,y2,z2)=path2[n+1]
            if anzahl>0:            
                print dis,anzahl
                for m in range(1,anzahl):
                    x=x1+(x2-x1)*m/anzahl
                    y=y1+(y2-y1)*m/anzahl
                    z=z1+(z2-z1)*m/anzahl
                    new_point=(x,y,z)
                    path.insert(n+m+count_points, new_point)
                count_points=count_points+anzahl-1
            print path
        n=n+1
    return path

#change grid coordinates to m in world coordinate system
def path_grid_to_m(path,start,goal):
    data=np.ndarray(shape=(len(path)+2,3),dtype=float)
    for next in range(len(path)):
        (x,y,z)=path[next]
        data[next+1,0]=x*0.4+0.4
        data[next+1,1]=y*0.4+0.2
        data[next+1,2]=z*0.4+0.3  
    (sx,sy,sz)=start
    data[0,0]=sx
    data[0,1]=sy
    data[0,2]=sz
    (gx,gy,gz)=goal
    data[len(path)+1,0]=gx
    data[len(path)+1,1]=gy
    data[len(path)+1,2]=gz
    #arrange the data to use the function
    data = data.transpose()
    #print data
    return data

 #2. step interpolate the remainging corner points of the path by using different degrees of polynoms
def interpolation_polynom(path,grad):
#    data=np.ndarray(shape=(len(path),3),dtype=float)   #create an array of float type for the input points
#    #fill the array with the Pathdata
#    a=path[0]
#    b=path[1]
#    c=path[2]
#    for i in range(len(a)):
#        data[i,0]=a[i]
#        data[i,1]=b[i]
#        data[i,2]=c[i]
#    #arrange the data to use the function
#    data = data.transpose()
    #interpolate polynom degree 1
    if grad==1:
        tck, u= interpolate.splprep(path,k=1,s=10)
        path = interpolate.splev(np.linspace(0,1,200), tck)
    #interpolate polynom degree 2
    if grad==2:
        tck, u= interpolate.splprep(path,k=2,s=10)
        path = interpolate.splev(np.linspace(0,1,200), tck)
    #interpolate polynom degree 3
    if grad==3:
        tck, u= interpolate.splprep(path, w=None, u=None, ub=None, ue=None, k=3, task=0, s=0.3, t=None, full_output=0, nest=None, per=0, quiet=1)
        path = interpolate.splev(np.linspace(0,1,200), tck)
    return path

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
def a_star_search(graph, start, goal,mapdata):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    print start
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    while not frontier.empty():
        current = frontier.get()
        
        if current == goal:
            
            break
        for next in graph.neighbors(current,mapdata):
            
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current
                
    
    return came_from, cost_so_far
    
    
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
        if mapdata2[x,y,z]==0:
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
    def neighbors(self, id,mapdata):
        (x, y, z) = id
        results = [(x+1, y, z), (x, y-1, z), (x-1, y, z), (x, y+1, z),(x, y, z+1),(x, y, z-1)]
        results = filter(self.in_bounds, results)
        results = filter(self.passable, results)
        return results

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
        out=out+mapdata2[x,y,z]
    #returns only true, if all nodes checked in the array returned the value 0 which means no obstacle
    return out==0

#this function is need to get the path(in points, nodes) from the results of the pathfinding algorythm
def reconstruct_path(came_from, start, goal):
    current = goal
    #print current
    path = [current]
    while current != start:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


##RRT
def distance(a, b):
    (x1, y1, z1) = a
    (x2, y2, z2) = b
    return math.sqrt((x1-x2)**2+(y1-y2)**2+(z1-z2)**2)   

def Nearest(node, tree):
    nearest = tree[0]
    for next in tree:
        if distance(next,node)<distance(nearest,node):
            nearest = next
    return nearest
    
def Extend(node1, node2, step):
    dis=0
    dis = distance(node1, node2)
    if collision(node1, node2):
        if dis<step:
            step = dis
        (x1,y1,z1) = node1
        (x2,y2,z2) = node2
        #straight line between the 2 nodes
        x = x1+(x2-x1)*step/dis
        y = y1+(y2-y1)*step/dis
        z = z1+(z2-z1)*step/dis
        #round the result to get the array indexs 
        x = round(x,0)
        y = round(y,0)
        z = round(z,0)
        node = (x,y,z)
    else:
        node=node1
    return node
    
def reconstruct_path_2(TreeStart, TreeGoal, TreeStartBefore, TreeGoalBefore, start, goal, node):
    current_node=node
    path=[]
    path.append(current_node)
    while current_node != start:
        currentIndex=getNodeIndex(TreeStart, current_node)-1
        current_node=TreeStartBefore[currentIndex]
        path.append(current_node)
    path.reverse()
    current_node=node
    while current_node != goal:
        currentIndex=getNodeIndex(TreeGoal, current_node)-1
        current_node=TreeGoalBefore[currentIndex]
        path.append(current_node)
    return path
 
def getNodeIndex(tree1, node):
    i=0
    for next in tree1:  
        if next == node:
            return i
        i=i+1


def rrt_search(graph, start, goal, mapdata):
    x,y,z=mapdata.shape
    K=x*y*z*10
    step=4
    TreeStart=[]
    TreeStartBefore=[]
    TreeGoal=[]
    TreeGoalBefore=[]
    TreeStart.append(start)
    TreeGoal.append(goal)
    test_node=(-1,-1,-1)
    for i in range(K):
        if i % 2:
            TreeStart, TreeStartBefore, new_node = extend_tree(TreeStart, TreeStartBefore, step, x, y, z)
            if new_node != test_node:
                if new_node in TreeGoal:
                    path = reconstruct_path_2(TreeStart, TreeGoal, TreeStartBefore, TreeGoalBefore, start, goal, new_node)
                    return path
        else:
            TreeGoal, TreeGoalBefore, new_node = extend_tree(TreeGoal, TreeGoalBefore, step, x, y, z)
            if new_node != test_node:
                if new_node in TreeStart:
                    path = reconstruct_path_2(TreeStart, TreeGoal, TreeStartBefore, TreeGoalBefore, start, goal, new_node)
                    return path
            
def extend_tree(Tree, TreeBefore, step, x, y, z):
    xrandom=random.randint(0, x-1)
    yrandom=random.randint(0, y-1)
    zrandom=random.randint(0, z-1)
    random_node = (xrandom, yrandom, zrandom) #random node of grid, int values needed
    nearest_node = Nearest(random_node, Tree) #select the nearest node in tree to the random node
    while random_node==nearest_node:
        xrandom=random.randint(0, x-1)
        yrandom=random.randint(0, y-1)
        zrandom=random.randint(0, z-1)
        random_node = (xrandom, yrandom, zrandom)
        nearest_node = Nearest(random_node, Tree)
    new_node = Extend(nearest_node, random_node, step)  #
    if new_node != nearest_node:
        Tree.append(new_node)
        TreeBefore.append(nearest_node)
    else:
        new_node=(-1,-1,-1)
    return Tree, TreeBefore, new_node   

