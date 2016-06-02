# -*- coding: utf-8 -*-
"""
Created on Mon Nov 16 21:00:37 2015

@author: renchen
"""
import numpy as np
import time
def  findnearst(position, path):
    
    x=path[0]
    y=path[1]
    z=path[2]
    dx=position[0]-x
    dy=position[1]-y
    dz=position[2]-z
         
    distance=np.sqrt(dx*dx+dy*dy+dz*dz)  #caculate distance between position and every points in the path

    minindex=np.argmin(distance)

    p_near=np.transpose(path)[minindex]  #get the nearst point

    v_approx=p_near-position
    if minindex<199:
        p_near1=np.transpose(path)[minindex+1] # the point which next to the nearst point
        v_tangent = p_near1-p_near
        #v_tangent_nor=v_tangent/(np.sqrt(v_tangent[0]*v_tangent[0]+v_tangent[1]*v_tangent[1]+v_tangent[2]*v_tangent[2]))
        alfa=1     # 0<alfa<1  self defination
        beta=15     # 0<beta<1  self defination
        v_result = alfa*v_approx + beta*v_tangent
    else:
        alfa=1     # 0<alfa<1  self defination
        #beta=15     # 0<beta<1  self defination
        v_result = alfa*v_approx
    
    return v_result

def find_vref(position, pnear):
    v_approx=pnear-position
    if minindex<199:
        p_near1=np.transpose(path)[minindex+1] # the point which next to the nearst point
        v_tangent = p_near1-p_near
        #v_tangent_nor=v_tangent/(np.sqrt(v_tangent[0]*v_tangent[0]+v_tangent[1]*v_tangent[1]+v_tangent[2]*v_tangent[2]))
        alfa=1     # 0<alfa<1  self defination
        beta=15     # 0<beta<1  self defination
        v_result = alfa*v_approx + beta*v_tangent
    else:
        alfa=1     # 0<alfa<1  self defination
        #beta=15     # 0<beta<1  self defination
        v_result = alfa*v_approx
    
    return v_result

def find_nearp(position, path):
    x=path[0]
    y=path[1]
    z=path[2]
    dx=position[0]-x
    dy=position[1]-y
    dz=position[2]-z
         
    distance=np.sqrt(dx*dx+dy*dy+dz*dz)  #caculate distance between position and every points in the path

    minindex=np.argmin(distance)

    p_near=np.transpose(path)[minindex]  #get the nearst point
    return p_near
     