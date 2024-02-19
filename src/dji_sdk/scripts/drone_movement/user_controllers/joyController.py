#!/usr/bin/env python

# Author: Christos Georgiades
# Email: cgeorg15@ucy.ac.cy
# Date :07/07/2022
# license removed for brevity
import os, sys

import rospy
import threading
import time
import math
import datetime

from std_msgs.msg import String, Bool, Header, Int8, Float64
from dji_sdk.msg import ScanArea, MissionWaypoint, JoystickParams, Quaternion
from kios.msg import Telemetry, MissionDji, GpsInput, MissionCommandDJI
from dji_sdk.srv import SetJoystickMode, JoystickAction#, ObtainControlAuthority
from dji_sdk.srv import FlightTaskControl
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3Stamped, Vector3
from math import sin, cos, atan2, sqrt

from simple_pid import PID

sys.path.append('/home/jetson/Documents/aiders_osdk/src/dji_sdk/scripts/drone_movement/')
import flightCommons as flc

#import joy_smoothing as js

# pidAlt = PID(Kp=1, Ki=0.0, Kd=0.2, sample_time=0.02,  output_limits=(-2.0, 5.0))
# pidAlt.setpoint = 0     # value we are trying to achieve
# pidAlt(0)               # value we read

# pidDist = PID(Kp=0.35, Ki=0.00023, Kd=0.07, sample_time=0.02, output_limits=(-2.0, 5.0))
# pidDist.setpoint = 0    # value we are trying to achieve
# pidDist(0)              # value we read

printStep = 50
#dronename = None

RAD_2_DEG = 57.29577951
DEG_2_RAD = 0.01745329252
EARTH_RADIUS = 6378137.0

activeJoystickAction = JoystickParams()
activeJoystickAction.x = 0.0
activeJoystickAction.y = 0.0
activeJoystickAction.z = 0.0
activeJoystickAction.yaw = 0.0

activeJoystickAction_mutex = threading.Lock()

latest_joystick_input = JoystickParams()
latest_joystick_input_mutex = threading.Lock()

st = 0.0
countSub = 0
countPub = 0
elapsed_time = 0.0

controller_deadzone = 0.05

bearing = 0.0
altitude = 0.0
latitude = 0.0
longitude = 0.0
velocity = 0.0
def telemetryCB(telemetry):
    global altitude, latitude, longitude, bearing, velocity
    global motorsActive
    altitude = float(telemetry.altitude)
    latitude = float(telemetry.latitude)
    longitude = float(telemetry.longitude)
    bearing = float(telemetry.heading)
    velocity = float(telemetry.velocity)

    if altitude > 3.0:
        motorsActive = True


# def setupJoystickMode():
#     global joystickModeSrv

#     horizontal_mode = 1
#     vertical_mode = 0
#     yaw_mode = 1
#     hoorizontal_coordinate = 1
#     stable_mode = 1

#     #response = joystickModeSrv(horizontal_mode, vertical_mode, yaw_mode, hoorizontal_coordinate, stable_mode)

#     print('Set Joystick Mode Response:', response)


# controlAuthorityActive = False
# def obtainControlAuthority(enable_obtain):
#     global controlAuthorityActive
#     #Type: dji_sdk/ObtainControlAuthority
#     #Args: enable_obtain
#     try:
#         controlAuthoritySrv = rospy.ServiceProxy(dronename+'/obtain_release_control_authority', ObtainControlAuthority)
#         response = controlAuthoritySrv(enable_obtain)

#         controlAuthorityActive = enable_obtain

#         print('Desired Control Authority:', enable_obtain, 'controlAuthoritySrv response:', response)
#     except Exception as e:
#         print('obtainControlAuthority - ', e)


motorsActive = False
def takeOffTask():
    global motorsActive
    global dronename
    global taskCtrlSrv

    print('altitude:', altitude)
    print('motorsActive:', motorsActive)

    try:
        if altitude < 0.5 and not motorsActive:
            motorsActive = True
            print('Activating Motors...')
            resp = taskCtrlSrv(7, JoystickParams(), 0, 0.0, 0.0)
            print(resp.result)
            print('Motors active')
        elif altitude < 3.0 and motorsActive:
            motorsActive = False
            print('Landing...')
            resp = taskCtrlSrv(31, JoystickParams(), 0, 0.0, 0.0)
            print(resp.result)
            print('Landed')
            print('Deactivating Motors...')
            resp = taskCtrlSrv(8, JoystickParams(), 0, 0.0, 0.0)
            print(resp.result)
            print('Motors deactivated')
    except rospy.ServiceException as e:
        pass


def retToHomeTask():
    global motorsActive
    global taskCtrlSrv

    print('altitude:', altitude)
    print('motorsActive:', motorsActive)

    if altitude > 3.0:
        print('Returning home...')
        resp = taskCtrlSrv(1, JoystickParams(), 0, 0.0, 0.0)
        #motorsActive = False


#axes: [left-stick-x, left-stick-y, left-trigger, right-stick-x, right-stick-y, right-trigger, dpad-x, dpad-y]
#axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0]
#buttons: [A, B, X, Y, left-bumper, right-bumper, select, start, home, left-stick-click, right-stick-click]
#buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
buttonState = [0] * 11
def joystickInputCB(joyInput):
    #print("len(axes)", len(joyInput.axes))
    #print("len(buttons)", len(joyInput.buttons))
    #print(joyInput, '\n')

    horizontalVel = 10  # m/s 
    verticalVel = 5     # m/s
    yawRate = 60       # deg/s

    x = joyInput.axes[4] * horizontalVel
    y = joyInput.axes[3] * horizontalVel * -1
    z = joyInput.axes[1] * verticalVel
    yaw = joyInput.axes[0] * yawRate * -1

    if joyInput.buttons[6] > 0 and not buttonState[6]:
        buttonState[6] = 1
        print('Pressed Control Authority Button')


        flc.obtain_ControlAuthority(not flc.get_control_authority_state())

        if flc.get_control_authority_state():
            flc.setupJoystickMode(autonomous_mode=False)
        else:
            flc.setupJoystickMode(autonomous_mode=True)


    if joyInput.buttons[7] > 0 and not buttonState[7]:
        buttonState[7] = 1
        print('Pressed TAKE OFF/LAND Button')
        takeOffTask()
    if joyInput.buttons[8] > 0 and not buttonState[8]:
        buttonState[8] = 1
        print('Pressed Return To Home Button')
        retToHomeTask()

    if joyInput.buttons[6] < 1:
        buttonState[6] = 0
    if joyInput.buttons[7] < 1:
        buttonState[7] = 0
    if joyInput.buttons[8] < 1:
        buttonState[8] = 0

    #print(x, y, z, yaw)
    save_action(x, y, z, yaw)


def save_action(x, y, z, yaw):
    global currentJoystickAction
    global activeJoystickAction, activeJoystickAction_mutex

    global historyJoystickAction, sampleIndex, sampleSize

    actionParams = JoystickParams()
    actionParams.x = x
    actionParams.y = y
    actionParams.z = z
    actionParams.yaw = yaw

    with activeJoystickAction_mutex:
        activeJoystickAction = actionParams

    # for i in reversed(range(1, 10)):
    # 	historyJoystickAction[i] = historyJoystickAction[i-1]

    # historyJoystickAction[0] = activeJoystickAction


# def push_action_to_drone(joystick_action):
#     global st, countSub, countPub
#     global outJoystickAction
#     countPub += 1
#     st = time.time()

#     print('push_action_to_drone - joystick_action:', joystick_action)

#     #response = joystickActionSrv(activeJoystickAction)
#     response = joystickActionSrv(joystick_action)

#     et = time.time()

#     countSub += 1

#     if not response:
#         print('================')
#         print('joystickActionSrv response:', response)
#         print(joystick_action)

#     countSub -= 1


smoothing_mode = 1
def set_smoothing_mode(mode):
    global smoothing_mode
    print('\njoyController - set_smoothing_mode:', mode, '\n')
    smoothing_mode = mode.data
    smoothing_mode_Pub.publish(smoothing_mode)


#lerp_rate = 1.0
lerp_rate = 0.8
def set_lerp_rate(desired_lerp_rate):
    global lerp_rate, lerp_rate_Pub
    print('\njoyController - set_lerp_rate:', desired_lerp_rate, '\n')
    lerp_rate = desired_lerp_rate.data
    lerp_rate_Pub.publish(lerp_rate)


#	Deprecated for time-slerp, smooth_lerp()
#	
sampleSize = 10
empty_action = JoystickParams()
empty_action.x = 0.0
empty_action.y = 0.0
empty_action.z = 0.0
empty_action.yaw = 0.0
historyJoystickAction = [empty_action] * sampleSize

sampleSize = 10
a = 0.25
def smooth_moving_average(target_joystick_action):
    global historyJoystickAction
    global a

    outJoystickAction = JoystickParams()

    # shift previous actions
    for i in reversed(range(1, 10)):
        historyJoystickAction[i] = historyJoystickAction[i - 1]

    # duplicate the latest action
    historyJoystickAction[0] = target_joystick_action

    x = 0
    y = 0
    z = 0
    yaw = 0
    for i in range(10):
        x = x + pow(1 - a, i) * historyJoystickAction[i].x
        y = y + pow(1 - a, i) * historyJoystickAction[i].y
        z = z + pow(1 - a, i) * historyJoystickAction[i].z

    outJoystickAction = JoystickParams()
    outJoystickAction.x = a * x
    outJoystickAction.y = a * y
    outJoystickAction.z = a * z
    outJoystickAction.yaw = target_joystick_action.yaw
    #print('smooth_moving_average - smoothed_joystick_action:', outJoystickAction, '\n')
        
    return outJoystickAction


def lerp(a, b, t):
    """Linear interpolation between values a and b with parameter t."""
    return a + (b - a) * t


def smooth_lerp(start_joystick_action, target_joystick_action, delta_time): 
    global lerp_rate

    #print('last_joystick_action:', last_joystick_action)
    #if activeJoystickAction:
    #    print('target_joystick_action:', activeJoystickAction)

    smoothed_joystick_action = JoystickParams()

    if target_joystick_action.x != 0.0:  
        smoothed_joystick_action.x = lerp(start_joystick_action.x, target_joystick_action.x, lerp_rate * delta_time)
    else:
        smoothed_joystick_action.x = 0.0

    if target_joystick_action.y != 0.0:  
        smoothed_joystick_action.y = lerp(start_joystick_action.y, target_joystick_action.y, lerp_rate * delta_time)
    else:
        smoothed_joystick_action.y = 0.0

    if target_joystick_action.z != 0.0:  
        smoothed_joystick_action.z = lerp(start_joystick_action.z, target_joystick_action.z, lerp_rate * delta_time)
    else:
        smoothed_joystick_action.z = 0.0

    # smoothed_joystick_action.y = lerp(start_joystick_action.y, target_joystick_action.y, lerp_rate * delta_time)
    # smoothed_joystick_action.z = lerp(start_joystick_action.z, target_joystick_action.z, lerp_rate * delta_time)
    smoothed_joystick_action.yaw = target_joystick_action.yaw

    if abs(smoothed_joystick_action.x) < 0.05:
        smoothed_joystick_action.x = 0.0

    if abs(smoothed_joystick_action.y) < 0.05:
        smoothed_joystick_action.y = 0.0

    if abs(smoothed_joystick_action.z) < 0.05:
        smoothed_joystick_action.z = 0.0

    #print('smooth_lerp - smoothed_joystick_action:', smoothed_joystick_action, '\n')

    return smoothed_joystick_action


def round_joystick_action(joystick_action):
    joystick_action.x = round(joystick_action.x, 4)
    joystick_action.y = round(joystick_action.y, 4)
    joystick_action.z = round(joystick_action.z, 4)
    joystick_action.yaw = activeJoystickAction.yaw

    return joystick_action


def get_latest_joystick_input():
    global activeJoystickAction, activeJoystickAction_mutex
    
    joystick_action = JoystickParams()
    with activeJoystickAction_mutex:
        joystick_action.x = activeJoystickAction.x
        joystick_action.y = activeJoystickAction.y
        joystick_action.z = activeJoystickAction.z
        joystick_action.yaw = activeJoystickAction.yaw

    return joystick_action
        

def joystick_worker():
    global smoothed_joy_pub

    print('joystick_worker - Started')
    rate = rospy.Rate(50)

    # Get the current time
    prev_time = rospy.Time.now()

    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"smoothed_joy_data_{timestamp}.txt"

    # Check if the 'data' directory exists, and create it if not
    directory = "data"
    if not os.path.exists(directory):
        os.makedirs(directory)
     

    start_joystick_input = JoystickParams()
    with open(os.path.join(directory, filename), 'w') as file:
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            # Calculate delta time
            dt = (current_time - prev_time).to_sec()
            # Update previous time stamp
            prev_time = current_time

            #print('joystick_worker - dt:', dt)
            target_joystick_input = get_latest_joystick_input()

            if smoothing_mode == 0:
                joystick_action = smooth_moving_average(target_joystick_input)
            elif smoothing_mode == 1:
                joystick_action = smooth_lerp(start_joystick_input, target_joystick_input , dt)
                start_joystick_input = joystick_action

            if joystick_action:
                joystick_action = round_joystick_action(joystick_action)
                
                file.write(str(joystick_action.x)+','+str(joystick_action.y)+','+str(joystick_action.z)+','+str(joystick_action.yaw) + '\n')

                flc.publishAction(joystick_action.x, joystick_action.y, joystick_action.z, joystick_action.yaw)

                smoothed_joy_packet = Quaternion()
                smoothed_joy_packet.x = joystick_action.x
                smoothed_joy_packet.y = joystick_action.y
                smoothed_joy_packet.z = joystick_action.z
                smoothed_joy_packet.w = joystick_action.yaw
                smoothed_joy_pub.publish(smoothed_joy_packet)

                if not (joystick_action.x == 0.0 and joystick_action.y == 0.0 and joystick_action.z == 0.0 and joystick_action.yaw == 0.0):
                    #print('Final Joystick Action:', joystick_action, '\n')
                    pass

            rate.sleep()

    print('joystick_worker - END')


def init():
    global joystickModeSrv, joystickActionSrv, smoothed_joy_pub
    global taskCtrlSrv
    global joystick_worker_thread
    global smoothing_mode_Pub, lerp_rate_Pub
    global lerp_rate

    #joystickModeSrv=rospy.ServiceProxy(dronename + '/set_joystick_mode', SetJoystickMode)
    #setupJoystickMode()

    rospy.Subscriber(dronename + '/Telemetry', Telemetry, telemetryCB)
    rospy.Subscriber(dronename + '/joy', Joy, joystickInputCB)
    rospy.Subscriber('/joy', Joy, joystickInputCB)

    smoothed_joy_pub = rospy.Publisher(dronename+'/joy/smoothed_joy', Quaternion, queue_size=1)

    joystickActionSrv = rospy.ServiceProxy(dronename+'/joystick_action', JoystickAction)
    taskCtrlSrv = rospy.ServiceProxy(dronename + '/flight_task_control', FlightTaskControl)

    rospy.Subscriber(dronename + '/joy/Set_Smooth_Mode', Int8, set_smoothing_mode)
    smoothing_mode_Pub = rospy.Publisher(dronename + '/joy/Get_Smooth_Mode', Int8, queue_size=1, latch=True)
    smoothing_mode_Pub.publish(smoothing_mode)

    rospy.Subscriber(dronename + '/joy/Set_Lerp_Rate', Float64, set_lerp_rate)
    lerp_rate_Pub = rospy.Publisher(dronename + '/joy/Get_Lerp_Rate', Float64, queue_size=1, latch=True)
    lerp_rate_Pub.publish(lerp_rate)

    flc.init()

    joystick_worker_thread = threading.Thread(target=joystick_worker)
    joystick_worker_thread.start()

    print('JoyController - Init Ready')


def listener(dji_name = "/matrice300", as_module=False):
    global dronename
    global countSub, countPub, elapsed_time


    dronename = dji_name
    boardId = dji_name.split('_', 1)[1]

    nodename = dronename.replace('/', '') + '_joyController'
    print('JoyController', nodename)


    if not as_module:
        rospy.init_node(nodename)
        init()
        rospy.spin()
    else:
        print(nodename,'- Imported as a module. Call init when ready to initialize rostopics...')


print('Initializing Joy Controller Handler...')
f = open("/var/lib/dbus/machine-id", "r")
boardId = f.read().strip()[0:4]

dronename = '/matrice300' + '_' + boardId
    
if __name__ == '__main__':
    as_module = False
else:
    as_module = True

listener(dji_name=dronename, as_module=as_module)