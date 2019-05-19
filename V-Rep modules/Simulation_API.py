
import cv2
print(cv2.__version__)

# -*- coding: utf-8 -*-
"""
Created on Tue May 07 17:42:44 2019

@author: 0neL4
"""

import time
import sys
import math
import vrep

pi=math.pi

'------------------------------------------------------------------------'
'Set up planning platform'
from Planning_Algorithm import *

state_space, map_shown, name_map, numerical_map = creat_platform()
original_map, minkowsky_map, street_name_map, check_table = map_generate()


def gps2street(vpt):
    _pt = coordinates_vrep2python(vpt)
    street_num_inf = check_pt_inf(_pt, minkowsky_map, street_name_map, check_table)
    return street_num_inf


def pickUp(_passanger_street, mode=None, _traffic_info=np.zeros(22)):
    a, b = np.where(numerical_map == 0)
    print(a, b)
    c, d = np.where(numerical_map == _passanger_street)
    pt_start = (int(a), int(b))
    pt_goal = (int(c), int(d))
    result, _map_shown = A_star(pt_start, pt_goal, state_space, map_shown, numerical_map, _traffic_info)
    _path, _actions = path_1(result, pt_start, pt_goal, _map_shown, name_map, check_table, mode, "Pick Up")

    return _path, _actions


def deliver(_passanger_street, _destination, mode=None, _traffic_info=np.zeros(22)):
    a, b = np.where(numerical_map == _passanger_street)
    c, d = np.where(numerical_map == _destination)
    pt_start = (int(a), int(b))
    pt_goal = (int(c), int(d))
    result, _map_shown = A_star(pt_start, pt_goal, state_space, map_shown, numerical_map, _traffic_info)
    _path, _actions = path_1(result, pt_start, pt_goal, _map_shown, name_map, check_table, mode, "Deliver", [255, 0, 0])
    return _path, _actions


def back2garage(_destination, mode=None, _traffic_info=np.zeros(22)):
    a, b = np.where(numerical_map == _destination)
    c, d = np.where(numerical_map == 0)
    pt_start = (int(a), int(b))
    pt_goal = (int(c), int(d))
    result, _map_shown = A_star(pt_start, pt_goal, state_space, map_shown, numerical_map, _traffic_info)
    _path, _actions = path_1(result, pt_start, pt_goal, _map_shown, name_map, check_table, mode, "Back to Garage", [0, 255, 0])
    return _path, _actions

'End of planning platform'
'--------------------------------------------------------------------------'

clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to V-REP
vrep.simxFinish(clientID)

print('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to V-REP
if clientID!=-1:
    print ('Connected to remote API server')
else:
    print ('Failed connecting to remote API server')
    sys.exit('Could not connect')


vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot);

time.sleep(1)

    
erroCode,FLW_handle=vrep.simxGetObjectHandle(clientID,'joint_front_left_wheel#0',vrep.simx_opmode_oneshot_wait)
erroCode,FRW_handle=vrep.simxGetObjectHandle(clientID,'joint_front_right_wheel#0',vrep.simx_opmode_oneshot_wait)
erroCode,BLW_handle=vrep.simxGetObjectHandle(clientID,'joint_back_left_wheel#0',vrep.simx_opmode_oneshot_wait)
erroCode,BRW_handle=vrep.simxGetObjectHandle(clientID,'joint_back_right_wheel#0',vrep.simx_opmode_oneshot_wait)

vrep.simxSetJointTargetVelocity(clientID,FLW_handle,0, vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetVelocity(clientID,FRW_handle,0, vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetVelocity(clientID,BLW_handle,0, vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetVelocity(clientID,BRW_handle,0, vrep.simx_opmode_oneshot)

vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);
time.sleep(1)
print("Sim Restarted")

errorGetProx,Prox=vrep.simxGetObjectHandle(clientID,'Ultrasonic',vrep.simx_opmode_oneshot_wait)

errorRead,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,Prox,vrep.simx_opmode_streaming)



#errorRead,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,Prox,vrep.simx_opmode_buffer)
[erroCode,simTime]=vrep.simxGetFloatSignal(clientID,'simulation_time',vrep.simx_opmode_streaming);

[erroCode,agent]=vrep.simxGetObjectHandle(clientID,'Agent',vrep.simx_opmode_oneshot_wait);
[erroCode,fastH]=vrep.simxGetObjectHandle(clientID,'fastHokuyo',vrep.simx_opmode_oneshot_wait);

[erroCode, position]=vrep.simxGetObjectPosition(clientID, agent,-1,vrep.simx_opmode_blocking)
[erroCode, orientation]=vrep.simxGetObjectOrientation(clientID,fastH,-1,vrep.simx_opmode_blocking)
ang=orientation[2]*180/pi
if ang<0:
    ang=ang+360
print(position)
print(ang)
Cang=ang

vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);
time.sleep(1)

STime=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
SL=np.array([10,10,10,10,4.7,10,10,10,10,7.3,10,10,10,10,10.1,10,10.1,10,10,10,10,10.1])
while True:

    vrep.simxSetJointTargetVelocity(clientID,FLW_handle,0, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetVelocity(clientID,FRW_handle,0, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetVelocity(clientID,BLW_handle,0, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetVelocity(clientID,BRW_handle,0, vrep.simx_opmode_streaming)

    streets=np.array([0,1,199,198,2,299,3,399,4,499,498,5,599,6,699,7,8,899,9,10,11,1199,12,1299,13,1399,1398,14,15,1599,16,1699,17,18,19,1999,20,2099,21,2199,22,2299,2298])
    gpsexit=np.array([(9999,9.4),(1.5,9999),(3.5,7),(5.5,9999),(15.5,9999),(17.5,9999),(27.5,9999),(9999,7),(9999,-3),(27.5,-5.05),(9999,-6.9),(9999,-11.6),(27.5,9999),(17.5,9999),(15.5,-3),(27.5,9999),(17.5,9999),(15.5,-14.2),(9999,7),(9999,-6.9),(5.5,9999),(3.45,-3),(15.5,9999),(17.5,-4.95),(5.5,9999),(3.45,-17),(1.5,9999),(9999,7),(9999,-6.9),(5.5,-4.95),(9999,-3),(1.5,-5.05),(9999,-17),(-8.5,9999),(1.5,9999),(3.5,-6.9),(-8.5,9999),(9999,-17),(9999,7),(-8.5,9.4),(9999,-6.9),(-8.5,-4.95),(9999,-3)])
    streetsDO=np.array([0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22])
    Corns=np.array([(-9.45,8.2),(-3.5,8.2),(10.5,8.2),(22.5,8.2),(28.45,2),(28.45,-9.25),(22.5,-3.975),(22.5,-5.975),(22.5,-12.9),(16.5,2),(16.5,-9.25),(10.5,-3.975),(10.5,-5.975),(10.5,-15.6),(4.5,2),(4.5,-9.25),(2.475,2),(2.475,-9.25),(-3.5,-3.975),(-3.5,-5.975),(-3.5,-17.95),(-9.45,2),(-9.45,-9.25)])
    streetsT=np.array([1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22])

    '--------------------------------------------------------------'
    'Set up'
    # gps_passanger = np.array(input("Enter the GPS loaction you are at (use 22.5,-3.975 as exmaple): ").split(",")).astype(float)
    # passanger_street = gps2street(gps_passanger)
    passanger_street = int(input("Enter in the street you are at: "))
    streetnumG = int(input("Enter in the street you would like to go to: "))
    speed = 4
    Usual_traffic = SL / speed
    Traffic_update = Usual_traffic.copy()

    print("traffic update")
    print(Traffic_update/Usual_traffic)

    'Pick Up'
    pick_path, pick_actions = pickUp(passanger_street, mode="show", _traffic_info=Traffic_update/Usual_traffic)
    pick_move = np.vstack((pick_path[:-1], pick_actions)).T
    for item in pick_move:
        Action = item[1]
        State = item[0]
        a = 1
        print("Speed is:")

        # speed = 3
        # print(speed)
        tspeed = 1
        ST = 1.15
        sr = 2

        ind = np.where(streets == State)
        GPS = gpsexit[ind]
        GPS = GPS[0]
        GPSX = GPS[0]
        GPSY = GPS[1]

        print("Next State")
        print(State)
        print(ind)
        print(GPSX)
        print(GPSY)
        [erroCode, position] = vrep.simxGetObjectPosition(clientID, agent, -1, vrep.simx_opmode_blocking)
        print(position)

        vrep.simxSetJointTargetVelocity(clientID, FLW_handle, 0, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, FRW_handle, 0, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, BLW_handle, 0, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, BRW_handle, 0, vrep.simx_opmode_streaming)

        if State == 599:
            Nang = 70
            TT = .3
        elif State == 899:
            Nang = 110
        elif State == 1399:
            Nang = 110
        else:
            Nang = 90
            TT = .71

        # Straight
        if Action == 1:
            TimeStart = time.time()
            print("SimTime")
            print(TimeStart)
            print(GPSX)
            print(GPSY)
            while not ((GPSX > position[0] - .1 and GPSX < position[0] + .1) or (
                    GPSY > position[1] - .1 and GPSY < position[1] + .1)):
                vrep.simxSetJointTargetVelocity(clientID, FLW_handle, speed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, FRW_handle, -speed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, BLW_handle, speed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, BRW_handle, -speed, vrep.simx_opmode_streaming)
                [erroCode, position] = vrep.simxGetObjectPosition(clientID, agent, -1, vrep.simx_opmode_blocking)
                # print(position)
                errorRead, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
                    clientID, Prox, vrep.simx_opmode_buffer)
                x, y, z = detectedPoint
                if x > .1 or y > .1:
                    print("Here")
                    if speed < 2:
                        speed = 2
                    else:
                        speed = speed - .1

            if State in streetsT:
                Tind = np.where(streetsT == State)
                Tind = Tind[0]
                Tind = Tind[0]
                print(Tind)
                EndTime = time.time()

                # DeltaSimTime = (EndTime - TimeStart) / SL[Tind]
                # if DeltaSimTime > STime[Tind]:
                #     STime[Tind] = DeltaSimTime
                DeltaSimTime = EndTime - TimeStart
                if DeltaSimTime > Usual_traffic[Tind]:
                    Traffic_update[Tind] = DeltaSimTime

        # Right
        if Action == 2:
            Gang = (-Nang + Cang + 360) % 360

            # Straight
            vrep.simxSetJointTargetVelocity(clientID, FLW_handle, speed, vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, FRW_handle, -speed, vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, BLW_handle, speed, vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, BRW_handle, -speed, vrep.simx_opmode_streaming)
            time.sleep(ST)
            # Right

            while not (Cang <= Gang + .5 and Cang >= Gang - .3):
                vrep.simxSetJointTargetVelocity(clientID, FLW_handle, tspeed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, FRW_handle, tspeed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, BLW_handle, tspeed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, BRW_handle, tspeed, vrep.simx_opmode_streaming)
                [erroCode, orientation] = vrep.simxGetObjectOrientation(clientID, fastH, -1, vrep.simx_opmode_blocking)
                Cang = -orientation[2] * 180 / pi
                print(Cang)
                if Cang < 0:
                    Cang = Cang + 360
                print(Cang)
                print(Gang)

            print(GPSX)
            print(GPSY)
            while not ((GPSX > position[0] - .1 and GPSX < position[0] + .1) or (
                    GPSY > position[1] - .1 and GPSY < position[1] + .1)):
                vrep.simxSetJointTargetVelocity(clientID, FLW_handle, speed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, FRW_handle, -speed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, BLW_handle, speed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, BRW_handle, -speed, vrep.simx_opmode_streaming)
                [erroCode, position] = vrep.simxGetObjectPosition(clientID, agent, -1, vrep.simx_opmode_blocking)
                # print(position)
                errorRead, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
                    clientID, Prox, vrep.simx_opmode_buffer)
                x, y, z = detectedPoint
                print(x)
                print(y)
                print(z)
                if x > .1 or y > .1:
                    print("Here")
                    if speed < 2:
                        speed = 2
                    else:
                        speed = speed - .1

        # Left
        if Action == 3:

            vrep.simxSetJointTargetVelocity(clientID, FLW_handle, speed, vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, FRW_handle, -speed, vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, BLW_handle, speed, vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, BRW_handle, -speed, vrep.simx_opmode_streaming)
            time.sleep(ST)

            Gang = (Nang + Cang + 360) % 360

            while not (Cang <= Gang + .3 and Cang >= Gang - 2.5):
                vrep.simxSetJointTargetVelocity(clientID, FLW_handle, -tspeed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, FRW_handle, -tspeed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, BLW_handle, -tspeed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, BRW_handle, -tspeed, vrep.simx_opmode_streaming)
                [erroCode, orientation] = vrep.simxGetObjectOrientation(clientID, fastH, -1, vrep.simx_opmode_blocking)
                Cang = -orientation[2] * 180 / pi
                if Cang < 0:
                    Cang = Cang + 360
                print(Cang)
                print(Gang)

            print(GPSX)
            print(GPSY)
            while not ((GPSX > position[0] - .1 and GPSX < position[0] + .1) or (
                    GPSY > position[1] - .1 and GPSY < position[1] + .1)):
                vrep.simxSetJointTargetVelocity(clientID, FLW_handle, speed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, FRW_handle, -speed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, BLW_handle, speed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, BRW_handle, -speed, vrep.simx_opmode_streaming)
                [erroCode, position] = vrep.simxGetObjectPosition(clientID, agent, -1, vrep.simx_opmode_blocking)
                # print(position)
                errorRead, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
                    clientID, Prox, vrep.simx_opmode_buffer)
                x, y, z = detectedPoint
                print(x)
                print(y)
                print(z)
                if x > .1 or y > .1:
                    print("Here")
                    if speed < 2:
                        speed = 2
                    else:
                        speed = speed - .1

            # Spin in Place
        if Action == 4:
            # Straight
            vrep.simxSetJointTargetVelocity(clientID, FLW_handle, speed, vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, FRW_handle, -speed, vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, BLW_handle, speed, vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, BRW_handle, -speed, vrep.simx_opmode_streaming)
            time.sleep(ST)
            # Right

            Gang = (180 + Cang + 360) % 360

            while not (Cang <= Gang + .4 and Cang >= Gang - .4):
                vrep.simxSetJointTargetVelocity(clientID, FLW_handle, tspeed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, FRW_handle, tspeed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, BLW_handle, tspeed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, BRW_handle, tspeed, vrep.simx_opmode_streaming)
                [erroCode, orientation] = vrep.simxGetObjectOrientation(clientID, fastH, -1, vrep.simx_opmode_blocking)
                Cang = -orientation[2] * 180 / pi
                if Cang < 0:
                    Cang = Cang + 360
                print(Cang)
                print(Gang)
    print("Pick Up Complete")
    print("traffic update")
    print(Traffic_update/Usual_traffic)

    'Deliver'
    deliver_path, deliver_actions = deliver(passanger_street, streetnumG, mode="show", _traffic_info=Traffic_update/Usual_traffic)
    deliver_move = np.vstack((deliver_path[:-1], deliver_actions)).T
    for item in deliver_move:
        Action = item[1]
        State = item[0]
        a = 1
        print("Speed is:")

        # speed = 3
        # print(speed)
        tspeed = 1
        ST = 1.15
        sr = 2

        ind = np.where(streets == State)
        GPS = gpsexit[ind]
        GPS = GPS[0]
        GPSX = GPS[0]
        GPSY = GPS[1]

        print("Next State")
        print(State)
        print(ind)
        print(GPSX)
        print(GPSY)
        [erroCode, position] = vrep.simxGetObjectPosition(clientID, agent, -1, vrep.simx_opmode_blocking)
        print(position)

        vrep.simxSetJointTargetVelocity(clientID, FLW_handle, 0, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, FRW_handle, 0, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, BLW_handle, 0, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, BRW_handle, 0, vrep.simx_opmode_streaming)
        # time.sleep(.5)
        # input()

        # [code dist]=vrep.simxReadDistance(clientID,Prox,vrep.simx_opmode_buffer)
        # print(detectedPoint)
        # print(x)
        # print(y)
        # print(z)
        # x==0 and y==0 and z==0

        # Check if odd street

        if State == 599:
            Nang = 70
            TT = .3
        elif State == 899:
            Nang = 110
        elif State == 1399:
            Nang = 110
        else:
            Nang = 90
            TT = .71

        # Straight
        if Action == 1:
            TimeStart = time.time()
            print("SimTime")
            print(TimeStart)
            print(GPSX)
            print(GPSY)
            while not ((GPSX > position[0] - .1 and GPSX < position[0] + .1) or (
                    GPSY > position[1] - .1 and GPSY < position[1] + .1)):
                vrep.simxSetJointTargetVelocity(clientID, FLW_handle, speed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, FRW_handle, -speed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, BLW_handle, speed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, BRW_handle, -speed, vrep.simx_opmode_streaming)
                [erroCode, position] = vrep.simxGetObjectPosition(clientID, agent, -1, vrep.simx_opmode_blocking)
                # print(position)
                errorRead, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
                    clientID, Prox, vrep.simx_opmode_buffer)
                x, y, z = detectedPoint
                # print(x)
                # print(y)
                # print(z)
                if x > .1 or y > .1:
                    print("Here")
                    if speed < 2:
                        speed = 2
                    else:
                        speed = speed - .1

            if State in streetsT:
                Tind = np.where(streetsT == State)
                Tind = Tind[0]
                Tind = Tind[0]
                EndTime = time.time()

                # DeltaSimTime = (EndTime - TimeStart) / SL[Tind]
                # if DeltaSimTime > STime[Tind]:
                #     STime[Tind] = DeltaSimTime
                DeltaSimTime = EndTime - TimeStart
                if DeltaSimTime > Usual_traffic[Tind]:
                    Traffic_update[Tind] = DeltaSimTime


        # Right
        if Action == 2:
            Gang = (-Nang + Cang + 360) % 360

            # Straight
            vrep.simxSetJointTargetVelocity(clientID, FLW_handle, speed, vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, FRW_handle, -speed, vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, BLW_handle, speed, vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, BRW_handle, -speed, vrep.simx_opmode_streaming)
            time.sleep(ST)
            # Right

            while not (Cang <= Gang + .5 and Cang >= Gang - .3):
                vrep.simxSetJointTargetVelocity(clientID, FLW_handle, tspeed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, FRW_handle, tspeed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, BLW_handle, tspeed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, BRW_handle, tspeed, vrep.simx_opmode_streaming)
                [erroCode, orientation] = vrep.simxGetObjectOrientation(clientID, fastH, -1, vrep.simx_opmode_blocking)
                Cang = -orientation[2] * 180 / pi
                print(Cang)
                if Cang < 0:
                    Cang = Cang + 360
                print(Cang)
                print(Gang)

            print(GPSX)
            print(GPSY)
            while not ((GPSX > position[0] - .1 and GPSX < position[0] + .1) or (
                    GPSY > position[1] - .1 and GPSY < position[1] + .1)):
                vrep.simxSetJointTargetVelocity(clientID, FLW_handle, speed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, FRW_handle, -speed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, BLW_handle, speed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, BRW_handle, -speed, vrep.simx_opmode_streaming)
                [erroCode, position] = vrep.simxGetObjectPosition(clientID, agent, -1, vrep.simx_opmode_blocking)
                # print(position)
                errorRead, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
                    clientID, Prox, vrep.simx_opmode_buffer)
                x, y, z = detectedPoint
                print(x)
                print(y)
                print(z)
                if x > .1 or y > .1:
                    print("Here")
                    if speed < 2:
                        speed = 2
                    else:
                        speed = speed - .1

        # Left
        if Action == 3:

            vrep.simxSetJointTargetVelocity(clientID, FLW_handle, speed, vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, FRW_handle, -speed, vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, BLW_handle, speed, vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, BRW_handle, -speed, vrep.simx_opmode_streaming)
            time.sleep(ST)

            Gang = (Nang + Cang + 360) % 360

            while not (Cang <= Gang + .3 and Cang >= Gang - 2.5):
                vrep.simxSetJointTargetVelocity(clientID, FLW_handle, -tspeed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, FRW_handle, -tspeed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, BLW_handle, -tspeed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, BRW_handle, -tspeed, vrep.simx_opmode_streaming)
                [erroCode, orientation] = vrep.simxGetObjectOrientation(clientID, fastH, -1, vrep.simx_opmode_blocking)
                Cang = -orientation[2] * 180 / pi
                if Cang < 0:
                    Cang = Cang + 360
                print(Cang)
                print(Gang)

            print(GPSX)
            print(GPSY)
            while not ((GPSX > position[0] - .1 and GPSX < position[0] + .1) or (
                    GPSY > position[1] - .1 and GPSY < position[1] + .1)):
                vrep.simxSetJointTargetVelocity(clientID, FLW_handle, speed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, FRW_handle, -speed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, BLW_handle, speed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, BRW_handle, -speed, vrep.simx_opmode_streaming)
                [erroCode, position] = vrep.simxGetObjectPosition(clientID, agent, -1, vrep.simx_opmode_blocking)
                # print(position)
                errorRead, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
                    clientID, Prox, vrep.simx_opmode_buffer)
                x, y, z = detectedPoint
                print(x)
                print(y)
                print(z)
                if x > .1 or y > .1:
                    print("Here")
                    if speed < 2:
                        speed = 2
                    else:
                        speed = speed - .1

            # Spin in Place
        if Action == 4:
            # Straight
            vrep.simxSetJointTargetVelocity(clientID, FLW_handle, speed, vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, FRW_handle, -speed, vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, BLW_handle, speed, vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, BRW_handle, -speed, vrep.simx_opmode_streaming)
            time.sleep(ST)
            # Right

            Gang = (180 + Cang + 360) % 360

            while not (Cang <= Gang + .4 and Cang >= Gang - .4):
                vrep.simxSetJointTargetVelocity(clientID, FLW_handle, tspeed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, FRW_handle, tspeed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, BLW_handle, tspeed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, BRW_handle, tspeed, vrep.simx_opmode_streaming)
                [erroCode, orientation] = vrep.simxGetObjectOrientation(clientID, fastH, -1, vrep.simx_opmode_blocking)
                Cang = -orientation[2] * 180 / pi
                if Cang < 0:
                    Cang = Cang + 360
                print(Cang)
                print(Gang)
    print("Deliever Complete")
    print("traffic update")
    print(Traffic_update/Usual_traffic)

    back_path, back_actions = back2garage(streetnumG, mode="show", _traffic_info=Traffic_update/Usual_traffic)
    back_move = np.vstack((back_path, np.hstack((back_actions, 4)))).T
    for item in deliver_move:
        Action = item[1]
        State = item[0]
        a = 1
        print("Speed is:")

        # speed = 3
        # print(speed)
        tspeed = 1
        ST = 1.15
        sr = 2

        ind = np.where(streets == State)
        GPS = gpsexit[ind]
        GPS = GPS[0]
        GPSX = GPS[0]
        GPSY = GPS[1]

        print("Next State")
        print(State)
        print(ind)
        print(GPSX)
        print(GPSY)
        [erroCode, position] = vrep.simxGetObjectPosition(clientID, agent, -1, vrep.simx_opmode_blocking)
        print(position)

        vrep.simxSetJointTargetVelocity(clientID, FLW_handle, 0, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, FRW_handle, 0, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, BLW_handle, 0, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, BRW_handle, 0, vrep.simx_opmode_streaming)
        # time.sleep(.5)
        # input()

        # [code dist]=vrep.simxReadDistance(clientID,Prox,vrep.simx_opmode_buffer)
        # print(detectedPoint)
        # print(x)
        # print(y)
        # print(z)
        # x==0 and y==0 and z==0

        # Check if odd street

        if State == 599:
            Nang = 70
            TT = .3
        elif State == 899:
            Nang = 110
        elif State == 1399:
            Nang = 110
        else:
            Nang = 90
            TT = .71

        # Straight
        if Action == 1:
            TimeStart = time.time()
            print("SimTime")
            print(TimeStart)
            print(GPSX)
            print(GPSY)
            while not ((GPSX > position[0] - .1 and GPSX < position[0] + .1) or (
                    GPSY > position[1] - .1 and GPSY < position[1] + .1)):
                vrep.simxSetJointTargetVelocity(clientID, FLW_handle, speed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, FRW_handle, -speed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, BLW_handle, speed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, BRW_handle, -speed, vrep.simx_opmode_streaming)
                [erroCode, position] = vrep.simxGetObjectPosition(clientID, agent, -1, vrep.simx_opmode_blocking)
                # print(position)
                errorRead, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
                    clientID, Prox, vrep.simx_opmode_buffer)
                x, y, z = detectedPoint
                # print(x)
                # print(y)
                # print(z)
                if x > .1 or y > .1:
                    print("Here")
                    if speed < 2:
                        speed = 2
                    else:
                        speed = speed - .1

            if State in streetsT:
                Tind = np.where(streetsT == State)
                Tind = Tind[0]
                Tind = Tind[0]
                print(Tind)
                EndTime = time.time()

                # DeltaSimTime = (EndTime - TimeStart) / SL[Tind]
                # print(DeltaSimTime)
                # if DeltaSimTime > STime[Tind]:
                #     STime[Tind] = DeltaSimTime
                DeltaSimTime = EndTime - TimeStart
                if DeltaSimTime > Usual_traffic[Tind]:
                    Traffic_update[Tind] = DeltaSimTime

        # Right
        if Action == 2:
            Gang = (-Nang + Cang + 360) % 360

            # Straight
            vrep.simxSetJointTargetVelocity(clientID, FLW_handle, speed, vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, FRW_handle, -speed, vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, BLW_handle, speed, vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, BRW_handle, -speed, vrep.simx_opmode_streaming)
            time.sleep(ST)
            # Right

            while not (Cang <= Gang + .5 and Cang >= Gang - .3):
                vrep.simxSetJointTargetVelocity(clientID, FLW_handle, tspeed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, FRW_handle, tspeed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, BLW_handle, tspeed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, BRW_handle, tspeed, vrep.simx_opmode_streaming)
                [erroCode, orientation] = vrep.simxGetObjectOrientation(clientID, fastH, -1, vrep.simx_opmode_blocking)
                Cang = -orientation[2] * 180 / pi
                print(Cang)
                if Cang < 0:
                    Cang = Cang + 360
                print(Cang)
                print(Gang)

            print(GPSX)
            print(GPSY)
            while not ((GPSX > position[0] - .1 and GPSX < position[0] + .1) or (
                    GPSY > position[1] - .1 and GPSY < position[1] + .1)):
                vrep.simxSetJointTargetVelocity(clientID, FLW_handle, speed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, FRW_handle, -speed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, BLW_handle, speed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, BRW_handle, -speed, vrep.simx_opmode_streaming)
                [erroCode, position] = vrep.simxGetObjectPosition(clientID, agent, -1, vrep.simx_opmode_blocking)
                # print(position)
                errorRead, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
                    clientID, Prox, vrep.simx_opmode_buffer)
                x, y, z = detectedPoint
                print(x)
                print(y)
                print(z)
                if x > .1 or y > .1:
                    print("Here")
                    if speed < 2:
                        speed = 2
                    else:
                        speed = speed - .1

        # Left
        if Action == 3:

            vrep.simxSetJointTargetVelocity(clientID, FLW_handle, speed, vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, FRW_handle, -speed, vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, BLW_handle, speed, vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, BRW_handle, -speed, vrep.simx_opmode_streaming)
            time.sleep(ST)

            Gang = (Nang + Cang + 360) % 360

            while not (Cang <= Gang + .3 and Cang >= Gang - 2.5):
                vrep.simxSetJointTargetVelocity(clientID, FLW_handle, -tspeed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, FRW_handle, -tspeed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, BLW_handle, -tspeed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, BRW_handle, -tspeed, vrep.simx_opmode_streaming)
                [erroCode, orientation] = vrep.simxGetObjectOrientation(clientID, fastH, -1, vrep.simx_opmode_blocking)
                Cang = -orientation[2] * 180 / pi
                if Cang < 0:
                    Cang = Cang + 360
                print(Cang)
                print(Gang)

            print(GPSX)
            print(GPSY)
            while not ((GPSX > position[0] - .1 and GPSX < position[0] + .1) or (
                    GPSY > position[1] - .1 and GPSY < position[1] + .1)):
                vrep.simxSetJointTargetVelocity(clientID, FLW_handle, speed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, FRW_handle, -speed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, BLW_handle, speed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, BRW_handle, -speed, vrep.simx_opmode_streaming)
                [erroCode, position] = vrep.simxGetObjectPosition(clientID, agent, -1, vrep.simx_opmode_blocking)
                # print(position)
                errorRead, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(
                    clientID, Prox, vrep.simx_opmode_buffer)
                x, y, z = detectedPoint
                print(x)
                print(y)
                print(z)
                if x > .1 or y > .1:
                    print("Here")
                    if speed < 2:
                        speed = 2
                    else:
                        speed = speed - .1

            # Spin in Place
        if Action == 4:
            # Straight
            vrep.simxSetJointTargetVelocity(clientID, FLW_handle, speed, vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, FRW_handle, -speed, vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, BLW_handle, speed, vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(clientID, BRW_handle, -speed, vrep.simx_opmode_streaming)
            time.sleep(ST)
            # Right

            Gang = (180 + Cang + 360) % 360

            while not (Cang <= Gang + .4 and Cang >= Gang - .4):
                vrep.simxSetJointTargetVelocity(clientID, FLW_handle, tspeed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, FRW_handle, tspeed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, BLW_handle, tspeed, vrep.simx_opmode_streaming)
                vrep.simxSetJointTargetVelocity(clientID, BRW_handle, tspeed, vrep.simx_opmode_streaming)
                [erroCode, orientation] = vrep.simxGetObjectOrientation(clientID, fastH, -1, vrep.simx_opmode_blocking)
                Cang = -orientation[2] * 180 / pi
                if Cang < 0:
                    Cang = Cang + 360
                print(Cang)
                print(Gang)
    print("Back in Garage")
    print("traffic update")
    print(Traffic_update/Usual_traffic)
    '------------------------------------------------------------------'

    #print("WARNING YOU ARE ABOUT TO START THE SIMULATION MOVEMENT")

    # for D in range(0,len(entire_moves)):
    #     Action = Directions[D, 1]
    #     State = Directions[D, 0]

    # for item in entire_moves:
    #     Action = item[1]
    #     State = item[0]
    #     a=1
    #     print("Speed is:")
    #
    #     speed=3
    #     print(speed)
    #     tspeed=1
    #     ST=1.15
    #     sr=2
    #
    #     ind=np.where(streets==State)
    #     GPS=gpsexit[ind]
    #     GPS=GPS[0]
    #     GPSX=GPS[0]
    #     GPSY=GPS[1]
    #
    #     print("Next State")
    #     print(State)
    #     print(ind)
    #     print(GPSX)
    #     print(GPSY)
    #     [erroCode, position]=vrep.simxGetObjectPosition(clientID, agent, -1, vrep.simx_opmode_blocking)
    #     print(position)
    #
    #     vrep.simxSetJointTargetVelocity(clientID,FLW_handle,0, vrep.simx_opmode_streaming)
    #     vrep.simxSetJointTargetVelocity(clientID,FRW_handle,0, vrep.simx_opmode_streaming)
    #     vrep.simxSetJointTargetVelocity(clientID,BLW_handle,0, vrep.simx_opmode_streaming)
    #     vrep.simxSetJointTargetVelocity(clientID,BRW_handle,0, vrep.simx_opmode_streaming)
    #     #time.sleep(.5)
    #     #input()
    #
    #     #[code dist]=vrep.simxReadDistance(clientID,Prox,vrep.simx_opmode_buffer)
    #     #print(detectedPoint)
    #     #print(x)
    #     #print(y)
    #     #print(z)
    #     #x==0 and y==0 and z==0
    #
    #     #Check if odd street
    #
    #     if State==599:
    #         Nang=70
    #         TT=.3
    #     elif State==899:
    #         Nang=110
    #     elif State==1399:
    #         Nang=110
    #     else:
    #         Nang=90
    #         TT=.71
    #
    #
    #     #Straight
    #     if Action==1:
    #         TimeStart=time.time()
    #         print("SimTime")
    #         print(TimeStart)
    #         print(GPSX)
    #         print(GPSY)
    #         while not((GPSX > position[0]-.1 and GPSX < position[0]+.1) or (GPSY > position[1]-.1 and GPSY < position[1]+.1 )):
    #             vrep.simxSetJointTargetVelocity(clientID,FLW_handle,speed, vrep.simx_opmode_streaming)
    #             vrep.simxSetJointTargetVelocity(clientID,FRW_handle,-speed, vrep.simx_opmode_streaming)
    #             vrep.simxSetJointTargetVelocity(clientID,BLW_handle,speed, vrep.simx_opmode_streaming)
    #             vrep.simxSetJointTargetVelocity(clientID,BRW_handle,-speed, vrep.simx_opmode_streaming)
    #             [erroCode, position]=vrep.simxGetObjectPosition(clientID, agent,-1,vrep.simx_opmode_blocking)
    #             #print(position)
    #             errorRead,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,Prox,vrep.simx_opmode_buffer)
    #             x,y,z=detectedPoint
    #             #print(x)
    #             #print(y)
    #             #print(z)
    #             if x>.1 or y>.1:
    #                print("Here")
    #                if speed<2:
    #                 speed=2
    #                else:
    #                 speed=speed-.1
    #
    #         if State in streetsT:
    #             Tind=np.where(streetsT==State)
    #             Tind=Tind[0]
    #             Tind=Tind[0]
    #             print(Tind)
    #             EndTime=time.time()
    #
    #             DeltaSimTime=(EndTime-TimeStart)/ SL[Tind]
    #             print(DeltaSimTime)
    #             if DeltaSimTime>STime[Tind]:
    #                 STime[Tind]=DeltaSimTime
    #
    #
    #     #Right
    #     if Action==2:
    #         Gang=(-Nang+Cang+360)%360
    #
    #
    #         #Straight
    #         vrep.simxSetJointTargetVelocity(clientID,FLW_handle,speed, vrep.simx_opmode_streaming)
    #         vrep.simxSetJointTargetVelocity(clientID,FRW_handle,-speed, vrep.simx_opmode_streaming)
    #         vrep.simxSetJointTargetVelocity(clientID,BLW_handle,speed, vrep.simx_opmode_streaming)
    #         vrep.simxSetJointTargetVelocity(clientID,BRW_handle,-speed, vrep.simx_opmode_streaming)
    #         time.sleep(ST)
    #         #Right
    #
    #
    #
    #         while not(Cang<=Gang+.5 and Cang>=Gang-.3):
    #             vrep.simxSetJointTargetVelocity(clientID,FLW_handle,tspeed, vrep.simx_opmode_streaming)
    #             vrep.simxSetJointTargetVelocity(clientID,FRW_handle,tspeed, vrep.simx_opmode_streaming)
    #             vrep.simxSetJointTargetVelocity(clientID,BLW_handle,tspeed, vrep.simx_opmode_streaming)
    #             vrep.simxSetJointTargetVelocity(clientID,BRW_handle,tspeed, vrep.simx_opmode_streaming)
    #             [erroCode, orientation]=vrep.simxGetObjectOrientation(clientID,fastH,-1,vrep.simx_opmode_blocking)
    #             Cang=-orientation[2]*180/pi
    #             print(Cang)
    #             if Cang<0:
    #                Cang=Cang+360
    #             print(Cang)
    #             print(Gang)
    #
    #
    #
    #         print(GPSX)
    #         print(GPSY)
    #         while not((GPSX > position[0]-.1 and GPSX < position[0]+.1) or (GPSY > position[1]-.1 and GPSY < position[1]+.1 )):
    #             vrep.simxSetJointTargetVelocity(clientID,FLW_handle,speed, vrep.simx_opmode_streaming)
    #             vrep.simxSetJointTargetVelocity(clientID,FRW_handle,-speed, vrep.simx_opmode_streaming)
    #             vrep.simxSetJointTargetVelocity(clientID,BLW_handle,speed, vrep.simx_opmode_streaming)
    #             vrep.simxSetJointTargetVelocity(clientID,BRW_handle,-speed, vrep.simx_opmode_streaming)
    #             [erroCode, position]=vrep.simxGetObjectPosition(clientID, agent,-1,vrep.simx_opmode_blocking)
    #             #print(position)
    #             errorRead,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,Prox,vrep.simx_opmode_buffer)
    #             x,y,z=detectedPoint
    #             print(x)
    #             print(y)
    #             print(z)
    #             if x>.1 or y>.1:
    #                print("Here")
    #                if speed<2:
    #                 speed=2
    #                else:
    #                 speed=speed-.1
    #
    #     #Left
    #     if Action==3:
    #
    #         vrep.simxSetJointTargetVelocity(clientID,FLW_handle,speed, vrep.simx_opmode_streaming)
    #         vrep.simxSetJointTargetVelocity(clientID,FRW_handle,-speed, vrep.simx_opmode_streaming)
    #         vrep.simxSetJointTargetVelocity(clientID,BLW_handle,speed, vrep.simx_opmode_streaming)
    #         vrep.simxSetJointTargetVelocity(clientID,BRW_handle,-speed, vrep.simx_opmode_streaming)
    #         time.sleep(ST)
    #
    #
    #         Gang=(Nang+Cang+360)%360
    #
    #         while not(Cang<=Gang+.3 and Cang>=Gang-2.5):
    #             vrep.simxSetJointTargetVelocity(clientID,FLW_handle,-tspeed, vrep.simx_opmode_streaming)
    #             vrep.simxSetJointTargetVelocity(clientID,FRW_handle,-tspeed, vrep.simx_opmode_streaming)
    #             vrep.simxSetJointTargetVelocity(clientID,BLW_handle,-tspeed, vrep.simx_opmode_streaming)
    #             vrep.simxSetJointTargetVelocity(clientID,BRW_handle,-tspeed, vrep.simx_opmode_streaming)
    #             [erroCode, orientation]=vrep.simxGetObjectOrientation(clientID,fastH,-1,vrep.simx_opmode_blocking)
    #             Cang=-orientation[2]*180/pi
    #             if Cang<0:
    #                Cang=Cang+360
    #             print(Cang)
    #             print(Gang)
    #
    #
    #         print(GPSX)
    #         print(GPSY)
    #         while not((GPSX > position[0]-.1 and GPSX < position[0]+.1) or (GPSY > position[1]-.1 and GPSY < position[1]+.1 )):
    #             vrep.simxSetJointTargetVelocity(clientID,FLW_handle,speed, vrep.simx_opmode_streaming)
    #             vrep.simxSetJointTargetVelocity(clientID,FRW_handle,-speed, vrep.simx_opmode_streaming)
    #             vrep.simxSetJointTargetVelocity(clientID,BLW_handle,speed, vrep.simx_opmode_streaming)
    #             vrep.simxSetJointTargetVelocity(clientID,BRW_handle,-speed, vrep.simx_opmode_streaming)
    #             [erroCode, position]=vrep.simxGetObjectPosition(clientID, agent,-1,vrep.simx_opmode_blocking)
    #             #print(position)
    #             errorRead,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,Prox,vrep.simx_opmode_buffer)
    #             x,y,z=detectedPoint
    #             print(x)
    #             print(y)
    #             print(z)
    #             if x>.1 or y>.1:
    #                print("Here")
    #                if speed<2:
    #                 speed=2
    #                else:
    #                 speed=speed-.1
    #
    #         #Spin in Place
    #     if Action==4:
    #         #Straight
    #         vrep.simxSetJointTargetVelocity(clientID,FLW_handle,speed, vrep.simx_opmode_streaming)
    #         vrep.simxSetJointTargetVelocity(clientID,FRW_handle,-speed, vrep.simx_opmode_streaming)
    #         vrep.simxSetJointTargetVelocity(clientID,BLW_handle,speed, vrep.simx_opmode_streaming)
    #         vrep.simxSetJointTargetVelocity(clientID,BRW_handle,-speed, vrep.simx_opmode_streaming)
    #         time.sleep(ST)
    #         #Right
    #
    #
    #         Gang=(180+Cang+360)%360
    #
    #         while not(Cang<=Gang+.4 and Cang>=Gang-.4):
    #             vrep.simxSetJointTargetVelocity(clientID,FLW_handle,tspeed, vrep.simx_opmode_streaming)
    #             vrep.simxSetJointTargetVelocity(clientID,FRW_handle,tspeed, vrep.simx_opmode_streaming)
    #             vrep.simxSetJointTargetVelocity(clientID,BLW_handle,tspeed, vrep.simx_opmode_streaming)
    #             vrep.simxSetJointTargetVelocity(clientID,BRW_handle,tspeed, vrep.simx_opmode_streaming)
    #             [erroCode, orientation]=vrep.simxGetObjectOrientation(clientID,fastH,-1,vrep.simx_opmode_blocking)
    #             Cang=-orientation[2]*180/pi
    #             if Cang<0:
    #                Cang=Cang+360
    #             print(Cang)
    #             print(Gang)
    #
    #
    #
    #     print("Action Complete")
     
vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot);

time.sleep(1)
##        
#            
#            
#           
print("Good bye!")      
exit