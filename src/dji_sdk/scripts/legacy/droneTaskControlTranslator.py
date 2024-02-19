#!/usr/bin/env python

# Author: Andreas
# Email: anastasiou.antreas@ucy.ac.cy
# Date :26/03/2021

# license removed for brevity


import sys
from dji_sdk.msg import JoystickParams
from dji_sdk.srv import FlightTaskControl, DroneTaskControl
import rospy
import random
import math

dji_name = "matrice300"

def taskControlCallback(req):
	global dji_name

	try:
		taskCtrl = rospy.ServiceProxy(dji_name + '/flight_task_control', FlightTaskControl)

		if (req.task == req.TASK_GOHOME):
			resp = taskCtrl(1,JoystickParams(),0,0.0,0.0)
			print(resp.result)
			return resp.result, 0, 0, 0
		elif (req.task == req.TASK_TAKEOFF):
			resp = taskCtrl(4,JoystickParams(),0,0.0,0.0)
			print(resp.result)
			return resp.result, 0, 0, 0
		elif (req.task == req.TASK_LAND):
			resp = taskCtrl(30,JoystickParams(),0,0.0,0.0)
			print(resp.result)
			return resp.result, 0, 0, 0
	
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e




def listener():
	global dji_name

	rospy.wait_for_service(dji_name + '/flight_task_control') 
	service = rospy.Service(dji_name+'/drone_task_control', DroneTaskControl, taskControlCallback)
	
	rateHz = 1
	rate = rospy.Rate(rateHz)  
	rate.sleep()
	if not rospy.is_shutdown():
		rospy.spin()

'''
Main Function
'''
if __name__ == '__main__':
	try:
		rospy.init_node('jetson300TaskControl', anonymous=False)
		listener()
	except rospy.ROSInterruptException:
		pass

