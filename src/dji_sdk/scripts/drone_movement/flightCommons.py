#!/usr/bin/env python

# Author: Christos Georgiades
# Email: cgeorg15@ucy.ac.cy
# Date : 19/07/2023

# license removed for brevity

import rospy

import threading
from math import sin, cos, atan2, sqrt

from std_msgs.msg import Bool, String

from dji_sdk.msg import MissionWaypoint
from dji_sdk.msg import JoystickParams, MissionWaypointTask


from dji_sdk.srv import ObtainControlAuthority
from dji_sdk.srv import SetJoystickMode

from kios.msg import Telemetry
from geometry_msgs.msg import Vector3Stamped

from drone_mission import DroneMission



altitude = 0.0
longitude = 0.0
latitude = 0.0
yaw = 0.0
latest_tele = Telemetry()
def telemetryCB(tele):
    global altitude, longitude, latitude, yaw
    global latest_tele
    altitude = tele.altitude
    longitude = tele.longitude
    latitude = tele.latitude
    yaw = tele.heading

    latest_tele = tele
    #print('Got latest_tele:\n', str(latest_tele))


telemetry_source = ''
def telemetry_source_CB(tele):
    global telemetry_source
    telemetry_source = tele.data


def get_telemetry_source():
    global telemetry_source
    return telemetry_source


g_yaw = 0
g_pitch = 0
g_roll = 0
def GimbalOrientationCB(gimbal):
    global g_yaw, g_pitch, g_roll
    g_yaw = gimbal.vector.z
    g_pitch = gimbal.vector.y
    g_roll = gimbal.vector.x


def getCurrentWp():
    global altitude, longitude, latitude, yaw

    wp = MissionWaypoint()
    wp.latitude = latitude
    wp.longitude = longitude
    wp.altitude = altitude
    wp.target_yaw = yaw

    return wp

def get_telemetry():
    global dronename
    global initial_timestamp, home_latitude, home_longitude
    global latest_tele

    return latest_tele

    now = rospy.Time.now() 
    telePckt = Telemetry()
    
    telePckt.serialVersionUID = str(dronename)
    
    telePckt.rostime_secs = now.secs
    telePckt.rostime_nsecs = now.nsecs
    telePckt.flightTimeSecs = now.secs - initial_timestamp
    
    telePckt.batteryThreashold = 10
    telePckt.batteryPercentage = latest_tele.batteryPercentage
    
    telePckt.gpsSignal = latest_tele.gpsSignal
    telePckt.satelliteNumber = latest_tele.satelliteNumber
    
    telePckt.altitude = altitude
    telePckt.longitude = longitude
    telePckt.latitude = latitude
    telePckt.heading = yaw
    
    telePckt.velocity = latest_tele.velocity
    
    telePckt.homeLatitude = home_latitude
    telePckt.homeLongitude = home_longitude
    
    #telePckt.droneState = calcDroneState(telePckt.altitude)
    telePckt.droneState = ''
    telePckt.gimbalAngle = g_pitch

    return telePckt


def get_positioning():
    positioningPacket = Positioning()
    positioningPacket.serialVersionUID = dronename
    positioningPacket.heading = tele.heading
    positioningPacket.velocity = tele.velocity
    
    positioningPacket.altitude = tele.altitude
    positioningPacket.latitude = tele.latitude
    positioningPacket.longitude = tele.longitude
    
    positioningPacket.d_roll = d_roll
    positioningPacket.d_pitch = d_pitch
    positioningPacket.d_yaw = d_yaw
    
    positioningPacket.g_roll = g_roll
    positioningPacket.g_pitch = g_pitch
    positioningPacket.g_yaw = g_yaw

    return positioningPacket


haveControlAuthority = False
def controlAuthorityStateCB(controlAuthorityState):
    global haveControlAuthority
    haveControlAuthority = controlAuthorityState.data

def get_control_authority_state():
    global haveControlAuthority
    return haveControlAuthority

def obtain_ControlAuthority(enable_obtain):
    global ControlAuthorityPub
    
    thread_id = '[' + str(threading.current_thread().ident) + ']'
    print('\n' + str(thread_id),'- obtainControlAuthority:', enable_obtain)
    
    c = 0
    c_max = 5
    retry_rate = rospy.Rate(1)
    print('obtain_ControlAuthority - get_control_authority_state():', get_control_authority_state())
    print('obtain_ControlAuthority - enable_obtain:', enable_obtain)
    while get_control_authority_state() != enable_obtain and c < c_max and not rospy.is_shutdown():
        if not enable_obtain:
            publishAction(0, 0, 0, yaw)
    
        ControlAuthorityPub.publish(enable_obtain)
        
        if get_control_authority_state() == enable_obtain:
            print('\n' + str(thread_id),'- Set_ControlAuthority:', 'Success', str(c) + '/' + str(c_max))
        else:
            print('\n' + str(thread_id),'- Set_ControlAuthority:', 'Fail', str(c) + '/' + str(c_max))
        
        c += 1
        retry_rate.sleep()


#   Args: horizontal_mode vertical_mode yaw_mode horizontal_coordinate stable_mode
#   FlightController::JoystickMode
#   FlightController::HorizontalLogic::HORIZONTAL_VELOCITY,
#   FlightController::VerticalLogic::VERTICAL_VELOCITY,
#   FlightController::YawLogic::YAW_ANGLE,
#   FlightController::HorizontalCoordinate::HORIZONTAL_BODY,
#   FlightController::StableMode::STABLE_ENABLE,
#
#	autonomous_mode=True: Yaw values set the desired heading angle
#	autonomous_mode=False: Yaw values set rotation speed around its axis
def setupJoystickMode(autonomous_mode=True):
    global joystickModeSrv   

    horizontal_mode = 1
    vertical_mode = 0

    yaw_mode = 0
    if autonomous_mode:
        yaw_mode = 0
    else:
        yaw_mode = 1

    hoorizontal_coordinate = 1
    stable_mode = 1
    
    response = joystickModeSrv(horizontal_mode, vertical_mode, yaw_mode, hoorizontal_coordinate, stable_mode)
    print('Set Joystick Mode Response:', response)



def publishAction(x, y, z, yaw):
    actionParams = JoystickParams()
    actionParams.x = x
    actionParams.y = y
    actionParams.z = z
    actionParams.yaw = yaw
    

    #print(x, y, z, yaw)    
    #response = joystickActionSrv(actionParams)
    
    JoystickActionPub.publish(actionParams)
    

def calculateDistanceBearing(wp_final, wp_init=None):
    RAD_2_DEG = 57.29577951
    DEG_2_RAD = 0.01745329252
    EARTH_RADIUS = 6378137.0

    if not wp_init:
        wp_init = getCurrentWp()

    #print("calculateDistanceBearing")
    
    # Latitude and Longtitude of Start Point in [rad]
    phiS = wp_init.latitude * DEG_2_RAD
    lamdaS = wp_init.longitude * DEG_2_RAD
    
    # Latitude and Longtitude of End Point in [rad]
    phiT = wp_final.latitude * DEG_2_RAD
    lamdaT = wp_final.longitude * DEG_2_RAD

    # Difference of Latitude and Longtitude of two Points
    dPhi = phiT - phiS
    dLamda = lamdaT - lamdaS

    # Distance [m] between two Points - Haversine Formula
    a = sin(dPhi/2)**2 + cos(phiS)*cos(phiT)*(sin(dLamda)/2)**2
    c = 2*atan2(sqrt(a),sqrt(1-a))
    distanceWp = EARTH_RADIUS * c

    # Bearing [degrees] of the End Point from the Start Pointaltitude
    y = sin(dLamda)*cos(phiT)
    x = cos(phiS)*sin(phiT)-sin(phiS)*cos(phiT)*cos(dLamda)
    bearingWp = atan2(y,x)*RAD_2_DEG
    
    #print("calculateDistanceBearing", distanceWp, bearingWp)
    return distanceWp, bearingWp


homePositionSet = False
home_latitude = 0.0
home_longitude = 0.0
initial_timestamp = 0.0
def set_home_position_CB(home_pos):
    global dronename
    global homePositionSet, home_latitude, home_longitude, initial_timestamp

    try:
        initial_timestamp = home_pos.rostime_secs
        home_latitude = home_pos.latitude
        home_longitude = home_pos.longitude

        homePositionSet = True
    
    except:
        print('flight_commons: error getting home position')


def get_home_position():
    global homePositionSet, home_latitude, home_longitude, initial_timestamp

    wp = MissionWaypoint()

    if homePositionSet:
        wp.latitude = home_latitude
        wp.longitude = home_longitude
        wp.altitude = altitude
        wp.target_yaw = yaw
    else:
        print('flightCommons - Home position not set, returning empty Waypoint.')

    return wp


crps_state_enabled = False
def crps_state_CB(state_enable):
    global crps_state_enabled
    crps_state_enabled = state_enable.data

def get_crps_state():
    global crps_state_enabled
    return crps_state_enabled

def set_crps_state(state_enable):
    global crpsStatePub

    print('Setting CRPS state:', state_enable)
    crpsStatePub.publish(state_enable)
    

def init():
    global ControlAuthorityPub, JoystickActionPub
    global crpsStatePub
    global joystickModeSrv

    global initial_timestamp
    initial_timestamp = rospy.Time.now().secs

    if rospy.is_shutdown():
        print(nodename,'- init()','- rospy is not ready')
    else:
        rospy.Subscriber(dronename+'/Telemetry', Telemetry, telemetryCB)
        rospy.Subscriber(dronename+'/PSP/Source', String, telemetry_source_CB)


        rospy.Subscriber(dronename+'/gimbal_angle',Vector3Stamped, GimbalOrientationCB)

        # Joystick Mode Service
        rospy.wait_for_service(dronename+'/set_joystick_mode')
        joystickModeSrv=rospy.ServiceProxy(dronename+'/set_joystick_mode', SetJoystickMode)



        # Control Authority
        rospy.Subscriber(dronename+'/ControlAuthority/Set_State_Enable', Bool, controlAuthorityStateCB)
        ControlAuthorityPub = rospy.Publisher(dronename+'/ControlAuthority/Set_State_Enable', Bool, queue_size=1)

        JoystickActionPub = rospy.Publisher(dronename+'/Joystick/PublishJoystickAction', JoystickParams, queue_size=1)
        

        crpsStatePub = rospy.Publisher(dronename+'/crps/Request', Bool, queue_size=1)
        rospy.Subscriber(dronename+'/crps/State', Bool, crps_state_CB)


        # home position
        rospy.Subscriber(dronename+ '/Telemetry/HomePosition', Telemetry, set_home_position_CB)

        print(nodename,'- init()','- DONE')


def listener(dji_name = "/matrice300", as_module = False):
    global responsePub, joystickModeSrv, joystickActionSrv
    global dronename, nodename
    global rate
    global x, y, z, yaw
    global controlAuthoritySrv, controlAuthStatePub

    dronename = dji_name
    nodename = dronename.replace('/', '') + '_flight_commons'
    #print(nodename)

    if not as_module:
        rospy.init_node(nodename)
        init()
    else:
        print(nodename,'- Imported as a module. Call init when ready to initialize rostopics...')

    if not as_module:
        rospy.spin()


print('Initializing Flight Commons...')
f = open("/var/lib/dbus/machine-id", "r")
boardId = f.read().strip()[0:4]

dronename = '/matrice300' + '_' + boardId

if __name__ == '__main__':
    as_module = False
else:
    as_module = True

listener(dji_name=dronename, as_module=as_module)