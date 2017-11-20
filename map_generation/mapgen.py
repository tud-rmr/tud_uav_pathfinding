# -*- coding: utf-8 -*-
"""
Created on Fri Oct 30 14:52:27 2015
@author: lijinke
"""

from vrep_interface import vrep
import numpy as np
import time
import math
from copy import deepcopy
import os.path


#enlarge the area of barrier: if a coordinate have barrier around it, this coordinate will be count that have barrier on it.
def save(mapdata2):
    mapdata3=deepcopy(mapdata2)
    for index in range(1,28):
        for index2 in range(1,28):
            for index3 in range(1,8):
                if  ((mapdata2[index-1,index2+1,index3]==1) or (mapdata2[index-1,index2,index3+1]==1) or (mapdata2[index-1,index2,index3-1]==1) or(mapdata2[index-1,index2,index3]==1) or (mapdata2[index-1,index2-1,index3+1]==1) or (mapdata2[index-1,index2-1,index3-1]==1) or (mapdata2[index-1,index2-1,index3]==1) or (mapdata2[index,index2+1,index3+1]==1) or (mapdata2[index,index2+1,index3-1]==1) or (mapdata2[index,index2-1,index3]==1) or (mapdata2[index,index2,index3+1]==1) or (mapdata2[index,index2,index3-1]==1) or (mapdata2[index+1,index2-1,index3]==1) or (mapdata2[index,index2-1,index3+1]==1) or (mapdata2[index,index2-1,index3-1]==1) or (mapdata2[index+1,index2,index3]==1) or (mapdata2[index+1,index2+1,index3+1]==1) or (mapdata2[index+1,index2+1,index3-1]==1) or (mapdata2[index+1,index2+1,index3]==1) or (mapdata2[index+1,index2,index3+1]==1) or (mapdata2[index+1,index2,index3-1]==1) or (mapdata2[index,index2+1,index3]==1) or (mapdata2[index+1,index2-1,index3+1]==1) or (mapdata2[index+1,index2-1,index3-1]==1) or (mapdata2[index-1,index2+1,index3+1]==1) or (mapdata2[index-1,index2+1,index3-1]==1)):
                    mapdata3[index,index2,index3]=1
                    print ('check')
    return mapdata3

#create data of map fleetly with hundred sensors
def mapgen_fast(scene_name,x,y,z,clientID):
    if os.path.isfile('./mapdata/'+scene_name+'.txt'):
        fo = open('./mapdata/'+scene_name+'.txt', "rb")
        arr = np.load(fo)['arr_0']
        #data_string = fo.read();
        #print ("Read String is : ", str)
        # Close opend file
        fo.close()
        #arr = np.loads(data_string)
        return arr
    else:
        x0=x
        y0=y
        errorCode,sensor1=vrep.simxGetObjectHandle(clientID,'Sensor_1',vrep.simx_opmode_oneshot_wait)
        objectHandles=np.array([sensor1])
        #create hundred sensors
        sensor_data=np.zeros(shape=(10,10,1),dtype=int)
        for x in range(10):
            for y in range(10):
                returnCode,new_sensor_handle=vrep.simxCopyPasteObjects(clientID,objectHandles,vrep.simx_opmode_oneshot_wait)
                vrep.simxReadProximitySensor(clientID,new_sensor_handle[0],vrep.simx_opmode_streaming)
                sensor_data[x,y]=new_sensor_handle[0]
                x1=0.0+x*0.2
                y1=0.0+y*0.2
                z1=0.0
                vrep.simxSetObjectPosition (clientID,sensor_data[x,y],-1,(x1,y1,z1),vrep.simx_opmode_oneshot)

        #move sensors to every point of map
        xmax=x0/2
        ymax=y0/2
        zmax=z/0.2
        xmax=int(math.ceil(xmax))
        ymax=int(math.ceil(ymax))
        zmax=int(math.ceil(zmax))
        arr=np.zeros(shape=(xmax*10,ymax*10,zmax),dtype=int)
        for index in range(xmax):
            for index2 in range(ymax):
                for index3 in range(zmax):
                    for x in range(10):
                        for y in range(10):
                            #judge barrier
                            errorCode,detectionState1,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor (clientID,sensor_data[x,y],vrep.simx_opmode_buffer)
                            if (detectionState1):
                                arr[index*10+x,index2*10+y,index3]=1   #save result
                            else:
                                arr[index*10+x,index2*10+y,index3]=0
                    for x in range(10):
                        for y in range(10):
                            x1=0.0 + 2*index  + x*0.2
                            y1=0.0 + 2*index2 + y*0.2
                            z1=0.0 + 0.2*index3
                            vrep.simxSetObjectPosition (clientID,sensor_data[x,y],-1,(x1,y1,z1),vrep.simx_opmode_oneshot)
                    time.sleep(0.2)
        #arr=save(arr)
        data_string=np.ndarray.dumps(arr)
        #print ("Data string is:", data_string)
        fo = open('./mapdata/'+scene_name+'.txt', "wb")
        np.savez(fo,arr)
        #fo.write(data_string);
        fo.close()
        return arr
