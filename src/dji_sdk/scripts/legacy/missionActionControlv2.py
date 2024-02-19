#!/usr/bin/env python

# Author: Andreas
# Email: anastasiou.antreas@ucy.ac.cy
# Date :26/03/2021

# license removed for brevity


import sys
from dji_sdk.srv import StartWaypointV2Mission, StopWaypointV2Mission, PauseWaypointV2Mission, ResumeWaypointV2Mission, ObtainControlAuthority, MissionWpAction
import rospy
import random
import math

dji_name = "matrice300"

def missionActionCallback(req):
	global dji_name

	try:

		startMission = rospy.ServiceProxy(dji_name + '/waypointV2_startMission', StartWaypointV2Mission)
		stopMission = rospy.ServiceProxy(dji_name + '/waypointV2_stopMission', StopWaypointV2Mission)
		pauseMission = rospy.ServiceProxy(dji_name + '/waypointV2_pauseMission', PauseWaypointV2Mission)
		resumeMission = rospy.ServiceProxy(dji_name + '/waypointV2_resumeMission', ResumeWaypointV2Mission)
		
		if (req.action == req.ACTION_START):
			resp = startMission();
		elif (req.action == req.ACTION_STOP):
			resp = stopMission()
		elif (req.action == req.ACTION_PAUSE):
			resp = pauseMission()
		elif (req.action == req.ACTION_RESUME):
			resp = resumeMission()
		print(resp.result)
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e
	finally:
		return resp.result, 0, 0, 0


def listener():
	global dji_name

	rospy.wait_for_service(dji_name + '/waypointV2_startMission') 
	service = rospy.Service(dji_name+'/mission_waypoint_action', MissionWpAction, missionActionCallback)
	
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
		rospy.init_node('jetson300missionActionControl', anonymous=False)
		listener()
	except rospy.ROSInterruptException:
		pass

