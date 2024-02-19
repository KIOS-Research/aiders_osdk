#!/usr/bin/env python

# Author: Christos Georgiades
# Email: cgeorg15@ucy.ac.cy
# Date :07/07/2022

# license removed for brevity

import rospy
import threading
import time 

from std_msgs.msg import String, Bool
#from dji_sdk.msg import ScanArea, MissionWaypoint, 
from dji_sdk.msg import JoystickParams
from kios.msg import Telemetry
#from kios.msg import MissionDji, GpsInput, MissionCommandDJI

from dji_sdk.srv import SetJoystickMode, JoystickAction, ObtainControlAuthority
#from sensor_msgs.msg import Image
#from geometry_msgs.msg import Vector3Stamped, Vector3

from math import sin, cos, atan2, sqrt


haveControlAuthority = False
def ControlAuthorityStateCB(CA_State):
	global haveControlAuthority
	haveControlAuthority = CA_State.data


x = 0.0
y = 0.0
z = 0.0
yaw = 0.0
def JoystickActionCB(action):
	global x, y, z, yaw    
	x = action.x
	y = action.y
	z = action.z
	yaw = action.yaw
	#print(x, y, z, yaw)


# #    Args: horizontal_mode vertical_mode yaw_mode horizontal_coordinate stable_mode
# #    FlightController::JoystickMode
# #    FlightController::HorizontalLogic::HORIZONTAL_VELOCITY,
# #    FlightController::VerticalLogic::VERTICAL_VELOCITY,
# #    FlightController::YawLogic::YAW_ANGLE,
# #    FlightController::HorizontalCoordinate::HORIZONTAL_BODY,
# #    FlightController::StableMode::STABLE_ENABLE,
# #
# #	autonomous_mode=True: Yaw values set the desired heading angle
# #	autonomous_mode=False: Yaw values set rotation speed around its axis
# def setupJoystickMode(autonomous_mode=True):
# 	global joystickModeSrv   

# 	horizontal_mode = 1
# 	vertical_mode = 0

# 	yaw_mode = 0
# 	if autonomous_mode:
# 		yaw_mode = 0
# 	else:
# 		yaw_mode = 1

# 	hoorizontal_coordinate = 1
# 	stable_mode = 1
	
# 	response = joystickModeSrv(horizontal_mode, vertical_mode, yaw_mode, hoorizontal_coordinate, stable_mode)
# 	print('Set Joystick Mode Response:', response)


def publishAction(x, y, z, yaw):
	global joystickActionSrv
	
	actionParams = JoystickParams()
	actionParams.x = x
	actionParams.y = y
	actionParams.z = z
	actionParams.yaw = yaw
	
	response = joystickActionSrv(actionParams)
	#print('Joystick Action', response)


def init():
	global joystickModeSrv, joystickActionSrv

	rospy.Subscriber(dronename+'/ControlAuthority/Current_State', Bool, ControlAuthorityStateCB)
	rospy.Subscriber(dronename+'/Joystick/PublishJoystickAction', JoystickParams, JoystickActionCB)

	rospy.wait_for_service(dronename+'/set_joystick_mode')
	joystickModeSrv=rospy.ServiceProxy(dronename+'/set_joystick_mode', SetJoystickMode)

	rospy.wait_for_service(dronename+'/joystick_action')
	joystickActionSrv=rospy.ServiceProxy(dronename+'/joystick_action', JoystickAction)

	#setupJoystickMode()

	print(nodename,'- init()','- DONE')


def listener(dji_name = "/matrice300"):
	global dronename, nodename
	
	dronename = dji_name
	nodename = dronename.replace('/', '') + '_flightController'
	print(nodename)
	
	rospy.init_node(nodename)   

	init()
	
	rate = rospy.Rate(50)
	while not rospy.is_shutdown():
		#print('haveControlAuthority', haveControlAuthority)
		if haveControlAuthority:
			publishAction(x, y, z, yaw)
		rate.sleep()


if __name__ == '__main__':
	try:
		print('Initializing Flight Controller...')
		f = open("/var/lib/dbus/machine-id", "r")
		boardId = f.read().strip()[0:4]
	
		dronename = '/matrice300' + '_' + boardId
		listener(dronename)
	except rospy.ROSInterruptException:
		pass

