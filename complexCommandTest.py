# Copyright 2006-2015 Coppelia Robotics GmbH. All rights reserved. 
# marc@coppeliarobotics.com
# www.coppeliarobotics.com
# 
# -------------------------------------------------------------------
# THIS FILE IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
# WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
# AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
# DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
# MISUSING THIS SOFTWARE.
# 
# You are free to use/modify/distribute this file for whatever purpose!
# -------------------------------------------------------------------
#
# This file was automatically created for V-REP release V3.2.2 Rev1 on September 5th 2015

# This example illustrates how to execute complex commands from
# a remote API client. You can also use a similar construct for
# commands that are not directly supported by the remote API.
#
# Load the demo scene 'remoteApiCommandServerExample.ttt' in V-REP, then 
# start the simulation and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!

def getCmdString(id,cnt,data):
    l=12+len(data)
    retData=vrep.simxPackInts([id,cnt,l])
    return retData+data

def waitForCmdReply(cnt):
    while True:
        result,string=vrep.simxReadStringStream(clientID,'repliesToRemoteApiClient',vrep.simx_opmode_streaming)
        if result==vrep.simx_return_ok:
            while len(string)!=0:
                headerPacked=string[0:12]
                header=vrep.simxUnpackInts(headerPacked)
                if cnt==header[1]:
                    replyData=''
                    if header[2]>12:
                        replyData=string[12:header[2]]
                    return replyData
                string=string[header[2]:len(string)]
try:
    import vrep
except:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')

import sys
import ctypes
print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP
if clientID!=-1:
    print ('Connected to remote API server')

    # Commands are send via the string signal 'commandsFromRemoteApiClient'.
    # Commands are simply appended to that string signal
    # Each command is coded in following way:
    # 1. Command ID (integer, 4 bytes)
    # 2. Command counter (integer, 4 bytes). Simply start with 0 and increment for each command you send
    # 3. Command length (integer, includes the command ID, the command counter, the command length, and the additional data (i.e. command data))
    # 4. Command data (chars/bytes, can be of any length, depending on the command)
    #
    # Replies are coded in a same way. An executed command should reply with the same command counter.
    # 
    # Above simple protocol is just an example: you could use your own protocol! (but it has to be the same on the V-REP side)

    # 1. First send a command to display a specific message in a dialog box:
    cmdID=1 # this command id means: 'display a text in a message box'
    cmdCnt=0
    cmdData=b'Hello world!'
    dataToSend=getCmdString(cmdID,cmdCnt,cmdData)
    vrep.simxWriteStringStream(clientID,'commandsFromRemoteApiClient',dataToSend,vrep.simx_opmode_oneshot)
    replyData=waitForCmdReply(cmdCnt)
    if sys.version_info[0] == 3:
        replyData=str(replyData,'utf-8')
    print ('Return string: ',replyData) # display the reply from V-REP (in this case, just a string)

    # 2. Now create a dummy object at coordinate 0.1,0.2,0.3:
    cmdID=2 # this command id means: 'create a dummy at a specific coordinate with a specific name'
    cmdCnt=cmdCnt+1
    cmdData=b'MyDummyName'+vrep.simxPackFloats([0.1,0.2,0.3])
    dataToSend=getCmdString(cmdID,cmdCnt,cmdData)
    vrep.simxWriteStringStream(clientID,'commandsFromRemoteApiClient',dataToSend,vrep.simx_opmode_oneshot)
    replyData=waitForCmdReply(cmdCnt)
    print ('Dummy handle: ',vrep.simxUnpackInts(replyData)[0]) # display the reply from V-REP (in this case, the handle of the created dummy)

    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')
