from math import *
import random
import time

import numpy as np
import math
from copy import deepcopy
import os.path

#import bpy
#
# Based on the great Course CS373 from Udacity taught by Sebastian Thrun
# https://www.udacity.com/course/cs373
#
# 3D Erweiterung von Paul Balzer
# CC-BY2.0 Lizenz

#Minor modifications by Raul Acuna @ TU Darmstadt

# Grid in 3D visualisieren


 
#def creategrid(grid, init, goal):
#    #Grid loschen
#    bpy.ops.object.select_by_type(type='MESH')
#    bpy.ops.object.delete(use_global=False)
#    
#    for z in range(len(grid)):
#        for y in range(len(grid[0])):
#            for x in range(len(grid[0][0])):
#                #print(grid[row][col])
#                if grid[z][y][x] == 1:
#                    bpy.ops.mesh.primitive_cube_add(location=(x,y,z), radius=.5)
#    print("Grid ready")
#
#creategrid(grid, init, goal)

# A* Algorithm
def search(goal, init, map_array):
    grid = np.asarray(map_array)
    grid = grid.transpose()
    grid = list(grid)
     
    heuristic = [[[0 for x in range(len(grid[0][0]))] for y in range(len(grid[0]))] for z in range(len(grid))]
     
    delta = [[-1, 0, 0], # zuruck
             [ 0,-1, 0], # links
             [ 1, 0, 0], # vor
             [ 0, 1, 0], # rechts
             [ 0, 0,-1], # unten
             [ 0, 0, 1]] # oben
    cost = 1
    start = time.clock()
    heuristic = calcheuristic(grid,goal,heuristic)
    print('Calcheuristic: %0.3fs' % (time.clock() - start))    
    
    closed = [[[0 for x in range(len(grid[0][0]))] for y in range(len(grid[0]))] for z in range(len(grid))]
    closed[init[0]][init[1]][init[2]] = 1
 
    expand = [[[-1 for x in range(len(grid[0][0]))] for y in range(len(grid[0]))] for z in range(len(grid))]
    action = [[[-1 for x in range(len(grid[0][0]))] for y in range(len(grid[0]))] for z in range(len(grid))]
 
 
    x = init[0]
    y = init[1]
    z = init[2]
    g = 0
    h = heuristic[z][y][x]
    f = g+h
 
    open = [[f, g, x, y, z]]
 
    found = False  # flag that is set when search is complete
    resign = False # flag set if we can't find expand
    count = 0
     
    while not found and not resign:
        if len(open) == 0:
            resign = True
            return "Fail"
        else:
            open.sort()
            open.reverse()
            next = open.pop()
             
            x = next[2]
            y = next[3]
            z = next[4]
            g = next[1]
            f = next[0]
            expand[z][y][x] = count
            count += 1
             
            if x == goal[0] and y == goal[1] and z == goal[2]:
                found = True
            else:
                for i in range(len(delta)):
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    z2 = z + delta[i][2]
                    if z2 >= 0 and z2 < len(grid) and \
                        y2 >=0 and y2 < len(grid[0]) and \
                        x2 >=0 and x2 < len(grid[0][0]):
                            if closed[z2][y2][x2] == 0 and grid[z2][y2][x2] == 0:
                                g2 = g + cost
                                f2 = g2 + heuristic[z2][y2][x2]
                                open.append([f2, g2, x2, y2, z2])
                                closed[z2][y2][x2] = 1
                                 
                                # Memorize the sucessfull action for path planning
                                action[z2][y2][x2] = i
                             
    path=[]
 
    path.append([goal[0], goal[1], goal[2]])
     
    while x != init[0] or y != init[1] or z != init[2]:
        x2 = x-delta[action[z][y][x]][0]
        y2 = y-delta[action[z][y][x]][1]
        z2 = z-delta[action[z][y][x]][2]
        #policy[x2][y2][z2]=delta_name[action[x][y][z]]
        x = x2
        y = y2
        z = z2
        # Path
        path.append([x2, y2, z2])

    #print('\nCoordinates for Path smoothing=')
    path.reverse()
     
    spath=smooth(path)
    
    return path
 
 
# Heuristic berechnen
def calcheuristic(grid,goal,heuristic):
    for z in range(len(grid)):
        for y in range(len(grid[0])):
            for x in range(len(grid[0][0])):
                # Euklidische Distanz fur jede Zelle zum Ziel berechnen
                dist=((x-goal[0])**2+(y-goal[1])**2+(z-goal[2])**2)**(1/2)
             
                heuristic[z][y][x]=dist
    return heuristic
 
def smooth(path, weight_data = 0.5, weight_smooth = 0.3, tolerance = 0.00001):
    # Make a deep copy of path into newpath
    newpath = [[0 for row in range(len(path[0]))] for col in range(len(path))]
    for i in range(len(path)):
        for j in range(len(path[0])):
            newpath[i][j] = path[i][j]
 
    change = tolerance
    while change >= tolerance:
        change = 0.0
        for i in range(1, len(path)-1): # 1. und letzten Punkt unberuhrt lassen
            for j in range(len(path[0])):
                           aux = newpath[i][j]
                           newpath[i][j] += weight_data * (path[i][j] - newpath[i][j])
                           newpath[i][j] += weight_smooth * (newpath[i-1][j] \
                                                             + newpath[i+1][j] - (2.0*newpath[i][j]))
                           change += abs(aux- newpath[i][j])
    for i in range(len(path)):
        print(path[i], newpath[i])
  
    return newpath