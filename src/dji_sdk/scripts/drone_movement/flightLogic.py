#!/usr/bin/env python

# Author: Christos Georgiades
# Email: cgeorg15@ucy.ac.cy
# Date :07/07/2022

# license removed for brevity

import rospy
import threading
import time 

from std_msgs.msg import String, Bool, Float32
#from dji_sdk.msg import ScanArea, MissionWaypoint, 
from dji_sdk.msg import MissionWaypoint
from dji_sdk.msg import JoystickParams, MissionWaypointTask
from kios.msg import Telemetry, MissionDji
#from kios.msg import MissionDji, GpsInput, MissionCommandDJI

#from dji_sdk.srv import SetJoystickMode, JoystickAction, obtain_ControlAuthority

#from sensor_msgs.msg import Image
#from geometry_msgs.msg import Vector3Stamped, Vector3

from math import sin, cos, atan2, sqrt
from simple_pid import PID

from autonomous_landing import vtolLogic as vtol
from autonomous_landing import lzLogic as lz

import flightCommons as flc
from drone_mission import DroneMission

controllerFrequency = 25
sampleTime = 1 / controllerFrequency
logicRate = None

pidAlt = PID(Kp=1, Ki=0.0, Kd=0.2, sample_time=sampleTime,  output_limits=(-2.0, 5.0))
pidAlt.setpoint = 0     # value we are trying to achieve
pidAlt(0)               # value we read

pidDist = PID(Kp=0.35, Ki=0.00023, Kd=0.07, sample_time=sampleTime, output_limits=(-2.0, 5.0))
pidDist.setpoint = 0    # value we are trying to achieve
pidDist(0)              # value we read

printStep = 50

responsePub=None
missionStateSub = None
droneStatePub = None
startMissionService = None

dronename = None
droneState='Landed'
cmd2Action = ['Start', 'Stop', 'Pause', 'Resume', 'Take-Off', 'Landing-Forced', 'Landing-LZ']
cmd2State = ['In_Mission', 'Flying', 'Paused_Mission', 'In_Mission', 'Taking-Off', 'Landing-Forced', 'Landing']

wpList = []
wpIndex = 0

altitude = 0.0
longitude = 0.0
latitude = 0.0
yaw = 0.0

bearingVal = 0.0


def set_velocity_range(vel_range):
    global pidAlt, pidDist

    if isinstance(vel_range, tuple):
        if len(vel_range) == 2:
            if vel_range[0] <= -1.0 and vel_range[1] >=1.0:
                pidAlt.output_limits = vel_range
                pidDist.output_limits = vel_range
                print('Velocity Range applied successful:', vel_range)
                return

    print('Not valid velocity range tuple:', vel_range)


default_vel_range=(-2.0, 5.0)
def reset_velocity_range():
    global default_vel_range
    set_velocity_range(default_vel_range)

# flightLogicState = 'Idle-Hover'
# def publishFlightLogicState():
#     global flightLogicStatePub, flightLogicState
    
#     flsPacket = String()
#     flsPacket.data = str(flightLogicState)
    
#     flightLogicStatePub.publish(flsPacket)
    

# def telemetryCB(tele):
#     global altitude, longitude, latitude, yaw
#     altitude = tele.altitude
#     longitude = tele.longitude
#     latitude = tele.latitude
#     yaw = tele.heading


# def getCurrentWp():
#     global altitude, longitude, latitude, yaw

#     wp = MissionWaypoint()
#     wp.latitude = latitude
#     wp.longitude = longitude
#     wp.altitude = altitude

#     return wp


minimumAltitude = 0.0
def minimumAltitudeCB(minAlt):
    #global minimumAltitude
    #minimumAltitude = minAlt.data
    set_minimum_altitude(minAlt.data)

def set_minimum_altitude(minAltitude):
    global minimumAltitude
    minimumAltitude = minAltitude

def iterate_wpList(missionTask):
    #while not flc.get_control_authority_state():
    #    flc.obtain_ControlAuthority(True)
    #   print('iterate_wpList -', 'get_control_authority_state:', flc.get_control_authority_state())
    
    

    thread_id = '[' + str(threading.current_thread().ident) + ']'
    print('\n' + str(thread_id),'- iterate_wpList')
    flc.obtain_ControlAuthority(True)
    wpMax = missionTask.get_waypoint_index_max() + 1

    print('missionTask.get_waypoint_finished():', missionTask.get_waypoint_finished())
    print('missionTask.get_mission_active():', missionTask.get_mission_active())
    while not missionTask.get_waypoint_finished() and missionTask.get_mission_active():
        wpIndex = missionTask.get_waypoint_index() + 1
        wp = missionTask.get_next_waypoint()
        print('\n' + str(thread_id),'- Traveling to waypoint ' + str(wpIndex) + '/' + str(wpMax))
        print(str(thread_id),'- a:'+str(round(wp.altitude,2)), 'lon:'+str(round(wp.longitude,6)), 'lat:'+str(round(wp.latitude,6))+'\n')

        goToAltitude(missionTask, wp)
        turnToYaw(missionTask, wp)
        coverDistance(missionTask, wp)
        print(str(thread_id),'- Traveling to waypoint ' + str(wpIndex) + '/' + str(wpMax) + '\tDONE')

        if missionTask.get_mission_active():
            missionTask.increment_waypoint_index()
        else:
            break
    
    print('\n' + str(thread_id),'- iterate_wpList DONE')
    flc.obtain_ControlAuthority(False)
    #missionTask.set_task_finished(True)


def check_takeoff(missionTask):
    print("check_takeoff() - flc.altitude:", round(flc.altitude, 2))

    if flc.altitude < 1.0:
        print('check_takeoff() - Altitude less than 1.0[m]. Taking Off')
        vtol.monitored_takeoff()

        flc.obtain_ControlAuthority(True)
        curWp = flc.getCurrentWp()
        curWp.altitude = 26.0
        
        goToAltitude(missionTask, curWp)
        flc.obtain_ControlAuthority(False)
    else:
        print('check_takeoff() - Altitude more than 1.0[m]. Will not call Take Off')
        #time.sleep(2)
        #obtain_ControlAuthority(False)
        #flc.obtain_ControlAuthority(True)


def traverseRoute(missionTask, args):
    check_takeoff(missionTask)

    thread_id = '[' + str(threading.current_thread().ident) + ']'
    
    
    print('\n' + str(thread_id),'- traverseRoute()')
    print(thread_id,'- missionTask:\n', str(missionTask))
    print(thread_id,'- args:\n', str(args), '\n')

    iterate_wpList(missionTask)

    print(thread_id,'- traverseRoute - Finished route')
    
    detect_and_land_lz = False
    print(thread_id,'- traverseRoute - Waypoint index:', str(missionTask.get_waypoint_index()) + '/' + str(missionTask.get_waypoint_index_max()))
    if wpIndex >= len(wpList):
        if detect_and_land_lz:
            landing_success = lz.landing_zone_landing()

            print(thread_id,'- Landing Success:', landing_success)

    if missionTask.get_mission_active():
        missionTask.set_task_finished(True)
        print(thread_id,'- === Route Traversed Successfully ===')
    else:
        #missionTask.set_task_finished(True)
        print(thread_id,'- === Route Traversal Paused ===')
    
    

def goToAltitude(missionTask, wp):
    thread_id = '[' + str(threading.current_thread().ident) + ']'

    def print_altitude_progress(altitude, wpAltitude, round_to=2):
        print(thread_id,'- Altitude: ' + str(round(altitude, round_to)) + '/' + str(round(wpAltitude, round_to)) + '[m]')

    global minimumAltitude  
    global flightLogicState
    
    
    
    altitudeError = 0.3       
    pidAlt.setpoint = wp.altitude
    
    task_polarity = True
    current_task_polarity = task_polarity
    if pidAlt.setpoint - flc.altitude > 0:
        flightLogicState = 'Ascend'
        task_polarity = True
    else:
        flightLogicState = 'Descend'
        task_polarity = False
    
    step =  0
    print_altitude_progress(flc.altitude, wp.altitude)
    while abs(pidAlt.setpoint - flc.altitude) > altitudeError and flc.altitude > minimumAltitude \
     and missionTask.get_mission_active() and not rospy.is_shutdown():
        if step > printStep:
            print_altitude_progress(flc.altitude, wp.altitude)
            step = 0
            print(thread_id,'- task_polarity:', task_polarity, 'current_task_polarity:', current_task_polarity)
        step += 1
        
        
        
        # if pidAlt.setpoint - flc.altitude > 0:
        #     current_task_polarity = True
        # else:
        #     current_task_polarity = False
            
        altV = pidAlt(flc.altitude)
        flc.publishAction(0, 0, altV, flc.yaw)
        logicRate.sleep()
               
    print_altitude_progress(flc.altitude, wp.altitude)


def turnToYaw(missionTask, wp):
    thread_id = '[' + str(threading.current_thread().ident) + ']'
    
    def print_yaw_progress(yaw, bearingWp, round_to=2):
        print(thread_id, '- Yaw: ' + str(round(yaw, round_to)) + '/' + str(round(bearingWp, round_to)) + '[deg]')

    global flightLogicState


    
    yawError = 0.2 
    print(thread_id, '- Turning to Yaw\n')
    
    _, bearingWp = flc.calculateDistanceBearing(wp)
    step =  0
    
    flightLogicState = 'Turning'
    print_yaw_progress(flc.yaw, bearingWp)
    while abs(bearingWp - flc.yaw) > yawError and missionTask.get_mission_active() and not rospy.is_shutdown():
        if step > printStep:
            print_yaw_progress(flc.yaw, bearingWp)
            step = 0
        step +=1
        
        _, bearingWp = flc.calculateDistanceBearing(wp)
            
            
        flc.publishAction(0, 0, 0, bearingWp)
        logicRate.sleep()

    print_yaw_progress(flc.yaw, bearingWp)


def coverDistance(missionTask, wp):
    thread_id = '[' + str(threading.current_thread().ident) + ']'

    def print_distance_progress(covered_distance, total_distance, round_to=2):
        print(thread_id, '- Distance: ' + str(round(covered_distance, round_to)) + '/' + str(round(total_distance, round_to)) + '[m]')
 
    global flightLogicState
    
    flightLogicState = 'Horizontal Movement'
    
    distanceError = 0.2
  
    total_distance, bearingWp = flc.calculateDistanceBearing(wp)
    current_distance = total_distance
    covered_distance = 0.0
    
    pidDist.setpoint = total_distance

    task_polarity = True
    current_task_polarity = task_polarity



    step =  0    
    print_distance_progress(covered_distance, total_distance)
    while total_distance - covered_distance > distanceError and covered_distance < total_distance \
    and missionTask.get_mission_active() and not rospy.is_shutdown() and task_polarity == current_task_polarity:  
        start = time.time()
        if step > printStep:
            print_distance_progress(covered_distance, total_distance)
            step = 0
            print(thread_id, '- task_polarity:', task_polarity, 'current_task_polarity:', current_task_polarity)
        step += 1
        
        current_distance, bearingWp = flc.calculateDistanceBearing(wp)
        distV = pidDist(covered_distance)
        
        flc.publishAction(distV, 0, 0, bearingWp)

        if total_distance - covered_distance > 0:
            current_task_polarity = True
        else:
            current_task_polarity = False
        
        logicRate.sleep()  
        end = time.time()
        covered_distance += distV * (end - start)

    print_distance_progress(covered_distance, total_distance)   


# def obtain_ControlAuthority(enable_obtain):
#     print('obtain_ControlAuthority:', enable_obtain)
    
#     if not enable_obtain:
#         publishAction(0, 0, 0, yaw)
    
#     ControlAuthorityPub.publish(enable_obtain)


# def publishAction(x, y, z, yaw):
#     actionParams = JoystickParams()
#     actionParams.x = x
#     actionParams.y = y
#     actionParams.z = z
#     actionParams.yaw = yaw
    

#     #print(x, y, z, yaw)    
#     #response = joystickActionSrv(actionParams)
    
#     JoystickActionPub.publish(actionParams)


# def initTasks():
#     print('Loading Mission manually')
#     waypoints = []
#     waypoint = MissionWaypoint()  

#     waypoint.latitude = 35.0987863447048084
#     waypoint.longitude = 33.42678843334993
#     waypoint.altitude = 28.0
#     waypoints.append(waypoint)
#     waypoint = MissionWaypoint()  

#     # waypoint.latitude = 35.10071296914134
#     # waypoint.longitude = 33.42721360216984
#     # waypoint.altitude = 28.0
#     # waypoints.append(waypoint)

#     # waypoint = MissionWaypoint()  

#     # waypoint.latitude = 35.10036471014452
#     # waypoint.longitude = 33.42635127836391
#     # waypoint.altitude = 28.0
#     # waypoints.append(waypoint)

#     # waypoint = MissionWaypoint()  

#     waypoint.latitude = 35.10168266172363
#     waypoint.longitude = 33.427037322472955
#     waypoint.altitude = 28.0
#     #waypoints.append(waypoint)
   
#     print('Waypoint Size: ', len(waypoints))
#     time.sleep(2)
#     initializeMission(waypoints)

def init():
    global droneStatePub, flightLogicStatePub
    global JoystickActionPub
    global logicRate, statePubRate

    logicRate = rospy.Rate(25)
    statePubRate = rospy.Rate(1)

    # Subscribe to Mission from platform
    #rospy.Subscriber(dronename+'/Mission', MissionDji, MissionCommandCB)
    #rospy.Subscriber(dronename+'/WaypointPath', MissionWaypointTask, initializeMission)
    
    #rospy.Subscriber(dronename+'/Telemetry', Telemetry, telemetryCB)  
    #rospy.Subscriber(dronename+'/setMinimumAltitude', Float32, minimumAltitudeCB)
    
    # Publish to flightController
    droneStatePub = rospy.Publisher(dronename+'/DroneState', String, queue_size=10)
    JoystickActionPub = rospy.Publisher(dronename+'/PublishJoystickAction', JoystickParams, queue_size=1)
    
    flightLogicStatePub = rospy.Publisher(dronename+'/FlightLogicState', String, queue_size=1)

    flc.init()

    vtol.init()
    lz.init()

    print(nodename, '- init() - DONE')


def listener(dji_name = "/matrice300", as_module=False):
    global dronename, nodename
    global logicRate
    
    dronename = dji_name  
    nodename = dronename.replace('/', '') + '_flightLogic'
    print(nodename)
    
    if not as_module:
        rospy.init_node(nodename)
        init()
    else:
        print(nodename,'- Imported as a module. Call init when ready to initialize rostopics...')

    if not as_module:
        rospy.spin()

        # while not rospy.is_shutdown():
        #     publishFlightLogicState()
        #     statePubRate.sleep()     


try:
    print('Initializing Flight Logic')
    f = open("/var/lib/dbus/machine-id", "r")
    boardId = f.read().strip()[0:4]

    dronename = '/matrice300' + '_' + boardId

    if __name__ == '__main__':
        as_module = False
    else:
        as_module = True

    listener(dronename, as_module=as_module)
except rospy.ROSInterruptException:
    pass

        
#================   
#('wp.latitude:', 35.10071296914134)
#('wp.longitude:', 33.42721360216984)
#('wp.altitude:', 28.84540557861328)
#================
#('wp.latitude:', 35.10036471014452)
#('wp.longitude:', 33.42635127836391)
#('wp.altitude:', 28.84540557861328)
#================
#('wp.latitude:', 35.10168266172363)
#('wp.longitude:', 33.427037322472955)
#('wp.altitude:', 28.84540557861328)
#================
#('wp.latitude:', 35.100863447048084)
#('wp.longitude:', 33.42638843334993)
#('wp.altitude:', 28.84540557861328)
