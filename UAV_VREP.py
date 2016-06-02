# -*- coding: utf-8 -*-
"""
Created on Tue Nov  3 15:31:37 2015

@author: lijinke
"""
import vrep#needed for the Connection with the Simulator
import numpy as np#needed for the arrays and some other mathematical operations
import time
import math
import pathfollowing

#get the position of an object
def getPosition(clientID,goal_new):
    errorcode,newgoal_handle=vrep.simxGetObjectHandle(clientID,goal_new,vrep.simx_opmode_oneshot_wait)
    #time.sleep(1) 
    errorCode,newgoal_position=vrep.simxGetObjectPosition(clientID,newgoal_handle,-1,vrep.simx_opmode_streaming)
    time.sleep(1) 
    errorCode,newgoal_position=vrep.simxGetObjectPosition(clientID,newgoal_handle,-1,vrep.simx_opmode_buffer)
    return newgoal_position
    
#calculate the angle between 2 vectors a and b
def angle_calculationx(a,b):
    dot = np.dot(a,b)
    x_modulus = np.sqrt(a[0]**2+a[1]**2+a[2]**2)
    y_modulus = np.sqrt(b[0]**2+b[1]**2+b[2]**2)
    cos_angle = dot / x_modulus / y_modulus 
    angle = np.arccos(cos_angle) #angle in radiant
    if b[1]>0:
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

#show the path in V-REP
def show_path2(path,clientID):
    
    errorCode,Graph=vrep.simxGetObjectHandle(clientID,'Graph',vrep.simx_opmode_oneshot_wait)
    
    print ("sending path to VREP")

    datax=path[0]
    datay=path[1]
    dataz=path[2]
    packedDatax=vrep.simxPackFloats(datax)
    vrep.simxClearStringSignal(clientID,'Path_Signalx',vrep.simx_opmode_oneshot)
    vrep.simxSetStringSignal(clientID,'Path_Signalx',packedDatax,vrep.simx_opmode_oneshot)
    packedDatay=vrep.simxPackFloats(datay)
    vrep.simxClearStringSignal(clientID,'Path_Signaly',vrep.simx_opmode_oneshot)
    vrep.simxSetStringSignal(clientID,'Path_Signaly',packedDatay,vrep.simx_opmode_oneshot)
    packedDataz=vrep.simxPackFloats(dataz)
    vrep.simxClearStringSignal(clientID,'Path_Signalz',vrep.simx_opmode_oneshot)
    vrep.simxSetStringSignal(clientID,'Path_Signalz',packedDataz,vrep.simx_opmode_oneshot)
        

        
def followPath2(clientID,path,goal):
    pathx=path[0]
    pathy=path[1]
    pathz=path[2]
    errorCode,UAV=vrep.simxGetObjectHandle(clientID,'UAV',vrep.simx_opmode_oneshot_wait)
    errorCode,pos=vrep.simxGetObjectPosition(clientID,UAV,-1,vrep.simx_opmode_streaming)
    errorCode,orientation=vrep.simxGetObjectOrientation(clientID,UAV,-1,vrep.simx_opmode_streaming)
    time.sleep(0.1) 
    errorCode,pos=vrep.simxGetObjectPosition(clientID,UAV,-1,vrep.simx_opmode_buffer)
    errorCode,orientation=vrep.simxGetObjectOrientation(clientID,UAV,-1,vrep.simx_opmode_buffer)
    xPosition=pos[0]
    yPosition=pos[1]
    zPosition=pos[2]
    vecp=[0,0,0]
    pdangle=0
    pveloz=0
    pangle=0
    pref_angz=0
    while (xPosition > pathx[199]+0.1) or (xPosition < pathx[199]-0.1) or (yPosition > pathy[199]+0.1) or (yPosition < pathy[199]-0.1) or (zPosition > pathz[199]+0.1) or (zPosition < pathz[199]-0.1):
        xvelomax=0.6
        yvelomax=0.6
        zmax=1
        absolut_dis=math.sqrt((xPosition-pathx[199])**2+(yPosition-pathy[199])**2+(zPosition-pathz[199])**2)
        #print absolut_dis
        slowvelo_dis=4
        if absolut_dis < slowvelo_dis:
            xvelomax=xvelomax*absolut_dis/slowvelo_dis
            yvelomax=yvelomax*absolut_dis/slowvelo_dis
            #zmax=zmax*absolut_dis/slowvelo_dis

        start_time = time.time()

        errorCode,pos=vrep.simxGetObjectPosition(clientID,UAV,-1,vrep.simx_opmode_buffer)
        errorCode,orientation=vrep.simxGetObjectOrientation(clientID,UAV,-1,vrep.simx_opmode_buffer)
        xPosition=pos[0]
        yPosition=pos[1]
        zPosition=pos[2]       
        
        # Find nearest point
        pnear =  pathfollowing.find_nearp(pos, path)               
        vec=pathfollowing.findnearst(pos,path)

        
        absolut=math.sqrt(vec[0]**2+vec[1]**2+vec[2]**2)
        xvelo=0
        yvelo=0
        height=zPosition
    
        xvelo_w=xvelomax*vec[0]/absolut#ref_velx
       
        yvelo_w=yvelomax*vec[1]/absolut#ref_vely
                
        height = pnear[2]
        
        #ref_angz, angle between (1/0/0) and (xvelo/yvelo/0)
        a1=[1,0,0]
        b1=[xvelo_w,yvelo_w,0]
        ref_angz=angle_calculationx(a1,b1)    
        if orientation[2]<0:
            angle=2*np.pi+orientation[2]
        else:
            angle=orientation[2]
        if ref_angz<0:
            ref_angz=2*np.pi+ref_angz
        dangle=angle-ref_angz
        
        if dangle>np.pi:
            veloz=6*(2*np.pi-dangle)/np.pi
        else:
            if dangle<-np.pi:
                veloz=-6*(2*np.pi+dangle)/np.pi
            else:
                veloz=-6*(dangle)/np.pi

        if dangle-pdangle>1:
            print (pdangle,dangle)
            print (pangle,angle)
            print (pref_angz,ref_angz)
            print (pveloz,veloz)
        pdangle=dangle
        pveloz=veloz
        pangle=angle
        pref_angz=ref_angz
        xvelo=xvelo_w*np.cos(-orientation[2])-yvelo_w*np.sin(-orientation[2])
        yvelo=xvelo_w*np.sin(-orientation[2])+yvelo_w*np.cos(-orientation[2])
        data=[xvelo,yvelo,height,0,0,veloz]

        packedData=vrep.simxPackFloats(data)
        vrep.simxClearStringSignal(clientID,'Command_Twist_Quad',vrep.simx_opmode_oneshot)
        vrep.simxSetStringSignal(clientID,'Command_Twist_Quad',packedData,vrep.simx_opmode_oneshot)

    data=[0,0,height,0,0,0]
    packedData=vrep.simxPackFloats(data)
    vrep.simxClearStringSignal(clientID,'Command_Twist_Quad',vrep.simx_opmode_oneshot)
    vrep.simxSetStringSignal(clientID,'Command_Twist_Quad',packedData,vrep.simx_opmode_oneshot)