# -*- coding: utf-8 -*-
"""
Created on Mon Nov 16 21:00:37 2015

@author: renchen
"""
import numpy as np
import time


def find_nearp(position, path):
    """ Finds and returns the closes point in the path to the given position"""
    x=path[0]
    y=path[1]
    z=path[2]
    dx=position[0]-x
    dy=position[1]-y
    dz=position[2]-z
    distance=np.sqrt(dx*dx+dy*dy+dz*dz)  #caculate distance between position and every points in the path
    minindex=np.argmin(distance)
    pnear=np.transpose(path)[minindex]  #get the closest path point
    return pnear, minindex


def  calc_vel_ref(position, path):
    """Calculates a velocity vector that approaches to the path using both
    an aproximation vector and a tangential vector"""

    pnear, minindex = find_nearp(position, path)
    v_approx=pnear-position
    if minindex<199:
        pnear1=np.transpose(path)[minindex+1] # the point which next to the nearst point
        v_tangent = pnear1-pnear
        alfa=0.01         # 0<alfa<1  self defination
        beta=(1-alfa)    # 0<beta<1  self defination
        v_result = alfa*v_approx + beta*v_tangent
    else:
        alfa=1     # 0<alfa<1  self defination
        v_result = alfa*v_approx
    return v_result

