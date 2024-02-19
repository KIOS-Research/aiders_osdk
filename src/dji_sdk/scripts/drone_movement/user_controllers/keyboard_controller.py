import os, sys

import rospy

import threading
from pynput import keyboard
from simple_pid import PID

from dji_sdk.msg import JoystickParams
from std_msgs.msg import Bool

project_path = os.path.realpath(__file__)
#print(project_path)

sys.path.append(project_path.split('user_controllers')[0])

#print(sys.path)

#from .. import flightCommons as flc
#from .. import flightCommons as flc
#from ..
import flightCommons as flc
import controller_commons as cc

pid_rate = 50
sample_time = 1 / pid_rate

pidAlt = PID(Kp=0.8, Ki=0.5, Kd=0.00001, sample_time=sample_time,  output_limits=(-5.0, 5.0))
pidAlt.setpoint = 0     # value we are trying to achieve
pidAlt(0)               # value we read

#0.1
#0.00023
30.07
pidYaw = PID(Kp=0.8, Ki=0.5, Kd=0.00001, sample_time=sample_time, output_limits=(-20.0, 20.0))
pidYaw.setpoint = 0    # value we are trying to achieve
pidYaw(0)              # value we read
#PID(Kp=0.35, Ki=0.00023, Kd=0.07, sample_time=sampleTime, output_limits=(-2.0, 5.0))

pidRoll = PID(Kp=0.8, Ki=0.5, Kd=0.00001, sample_time=sample_time, output_limits=(-5.0, 5.0))
pidRoll.setpoint = 0    # value we are trying to achieve
pidRoll(0)              # value we read

pidPitch = PID(Kp=0.8, Ki=0.5, Kd=0.00001, sample_time=sample_time, output_limits=(-5.0, 5.0))
pidPitch.setpoint = 0    # value we are trying to achieve
pidPitch(0)              # value we read




axis = {'1': 'Negative Altitude',
        '3': 'Positive Altitude',
        '7': 'Negative Yaw',
        '9': 'Positive Yaw',
        '4': 'Negative Roll',
        '6': 'Positive Roll',
	    '2': 'Negative Pitch',
        '8': 'Positive Pitch',
	    '5': 'Drone/Camera Modifier'}


sensitivity = 0.10
def augment_pid_setpoint(pid, direction):
    output_limits = pid.output_limits
    setpoint = pid.setpoint

    min_setpoint = output_limits[0]
    max_setpoint = output_limits[1]
    if direction == 1:
        pid.setpoint = setpoint + max_setpoint * sensitivity
        
        if pid.setpoint > max_setpoint:
            pid.setpoint = max_setpoint
    elif direction == -1:
        pid.setpoint = setpoint + min_setpoint * sensitivity

        if pid.setpoint < min_setpoint:
            pid.setpoint = min_setpoint

    #print(pid, '- setpoint:', pid.setpoint)


def translate_to_action(key):
    print('key.char:', key.char)

    axis_string = axis[key.char]
    direction = 0
    pid = None

    print('axis_string:', axis_string)
    if 'Positive' in axis_string:
        direction = 1
    elif 'Negative' in axis_string:
        direction = -1

    if 'Altitude' in axis_string:
        pid = pidAlt
    elif 'Yaw' in axis_string:
        pid = pidYaw
    elif 'Roll' in axis_string:
        pid = pidRoll
    elif 'Pitch' in axis_string:
        pid = pidPitch

    
    if pid and direction != 0:
        #print(pid, direction)
        augment_pid_setpoint(pid, direction)
        print(pidPitch.setpoint, pidRoll.setpoint, pidAlt.setpoint, pidYaw.setpoint)
    else:
        #flc.obtain_ControlAuthority(True)
        handle_modifier_key()

def handle_modifier_key():
    print('handle_modifier_key')
    have_control_authority = flc.get_control_authority_state()
    cc.setupJoystickMode(not have_control_authority)
    flc.obtain_ControlAuthority(not have_control_authority)

def handle_key_on_press(key):
    try:
        if key.vk and key.vk == 65437:
            key.char = '5'

        if key.char in axis:
            translate_to_action(key)
        else:
            print('key:', key)
            print('key.char else:',key.char)

    except AttributeError: 
        pass

def on_press(key):
    handle_key_on_press(key)


def on_release(key):
   pass

print_c = 0
print_max_c = 25
def update_joystick(actionParams):
    global print_c, print_max_c
    y = actionParams.y
    actionParams_new = JoystickParams()
    actionParams_new.x = pidPitch(actionParams.x)
    actionParams_new.y = pidRoll(actionParams.y)
    actionParams_new.z = pidAlt(actionParams.z)
    actionParams_new.yaw = pidYaw(actionParams.yaw)

    print_c = print_c + 1
    #if print_c > print_max_c:
        #print(actionParams.x, actionParams.y, actionParams.z, actionParams.yaw)
    #    print('y:', round(y,4), 'actionParams.y', round(actionParams_new.y, 4), 'pidRoll.setpoint:', pidRoll.setpoint)
    #    print_c = 0

    return actionParams_new

def action_publisher_worker():
    rate = rospy.Rate(pid_rate)
    actionParams = JoystickParams()
    actionParams.x = 0.0
    actionParams.y = 0.0
    actionParams.z = 0.0
    actionParams.yaw = 0.0

    while not rospy.is_shutdown():
        actionParams = update_joystick(actionParams)
        #print(actionParams.x, actionParams.y, actionParams.z, actionParams.yaw)
        flc.publishAction(actionParams.x, actionParams.y, actionParams.z, actionParams.yaw)
        
        rate.sleep()


def init():
    if rospy.is_shutdown():
        print(nodename,'- init()','- rospy is not ready')
    else:
        flc.init()
        cc.init()



        action_publisher = threading.Thread(target=action_publisher_worker)
        action_publisher.start()

        kbd_listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        kbd_listener.start()

        print(nodename,'- init()','- DONE')


def listener(dji_name = "/matrice300", as_module = False):
    global dronename, nodename

    dronename = dji_name
    nodename = dronename.replace('/', '') + '_keyboard_controller'
    #print(nodename)

    if not as_module:
        rospy.init_node(nodename)
        init()
    else:
        print(nodename,'- Imported as a module. Call init when ready to initialize rostopics...')

    if not as_module:
        rospy.spin()


print('Initializing Keyboard Controller...')
f = open("/var/lib/dbus/machine-id", "r")
boardId = f.read().strip()[0:4]

dronename = '/matrice300' + '_' + boardId

if __name__ == '__main__':
    as_module = False
else:
    as_module = True

listener(dji_name=dronename, as_module=as_module)