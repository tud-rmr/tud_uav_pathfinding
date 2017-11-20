# -*- coding: utf-8 -*-
"""
Created on Tue Mar 29 14:51:04 2016

@author: lracuna
"""
import random
import math
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
    
def Extend(node1, node2, step, mapdata):
    dis=0
    dis = distance(node1, node2)
    if collision(node1, node2, mapdata):
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


def search(graph, start, goal, mapdata):
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
            TreeStart, TreeStartBefore, new_node = extend_tree(TreeStart, TreeStartBefore, step, x, y, z,mapdata)
            if new_node != test_node:
                if new_node in TreeGoal:
                    path = reconstruct_path_2(TreeStart, TreeGoal, TreeStartBefore, TreeGoalBefore, start, goal, new_node)
                    return path
        else:
            TreeGoal, TreeGoalBefore, new_node = extend_tree(TreeGoal, TreeGoalBefore, step, x, y, z,mapdata)
            if new_node != test_node:
                if new_node in TreeStart:
                    path = reconstruct_path_2(TreeStart, TreeGoal, TreeStartBefore, TreeGoalBefore, start, goal, new_node)
                    return path
            
def extend_tree(Tree, TreeBefore, step, x, y, z, mapdata):
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
    new_node = Extend(nearest_node, random_node, step, mapdata)  #
    if new_node != nearest_node:
        Tree.append(new_node)
        TreeBefore.append(nearest_node)
    else:
        new_node=(-1,-1,-1)
    return Tree, TreeBefore, new_node   

def collision(a,b, mapdata):
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
        out=out+mapdata[x,y,z]
    #returns only true, if all nodes checked in the array returned the value 0 which means no obstacle
    return out==0