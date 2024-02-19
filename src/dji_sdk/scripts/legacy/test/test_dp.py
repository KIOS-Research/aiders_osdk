#!/usr/bin/env python

# Author: Andreas
# Email: anastasiou.antreas@ucy.ac.cy
# Date :10/02/2020

# license removed for brevity

import rospy
import traceback
import sys
from termcolor import colored
import re
#import tf
from multiprocessing import Process
from dji_sdk.msg import ScanArea, MissionWaypoint, MissionWaypointTask
from dji_sdk.srv import MissionWpAction
from dph import Path
from std_msgs.msg import String, Int8
responsePub = None
tempPub = None
mission1 = None


def scan_callback(mission):
	global mission1
	mission1 = mission
	ll, ul, lr, ur = mission.areaCorners
	test = Path(10,10, ll, ul, lr, ur)
	test.visited([0])
	batt = mission.batteries
	visitedNodes = [0 for i in range (0,100)]
	visitedNodes[0] = 1	
	circuitX, circuitY, vis = test.calculateCircuits(batt, 0, 0, mission.waypointTask.idle_velocity, visitedNodes)
	global responsePub
	response = MissionWaypointTask()
	response = mission.waypointTask
        response.mission_waypoint = []
	print "vis: ", vis
	for i in range(0,len(circuitX[mission.agentNumber])-1):
		#print circuitX[mission.agentNumber][i] , "," , circuitY[mission.agentNumber][i]
		waypoint = MissionWaypoint()		
		waypoint.latitude = circuitX[mission.agentNumber][i]
		waypoint.longitude = circuitY[mission.agentNumber][i]
		waypoint.altitude = mission.altitude
		waypoint.damping_distance = 3
		waypoint.target_yaw = 0
		waypoint.target_gimbal_pitch = 0
		waypoint.turn_mode = 0
		waypoint.has_action = 0
		waypoint.action_time_limit = 0
		print waypoint.latitude , " , ", waypoint.longitude, " , ", waypoint.altitude
		response.mission_waypoint.append(waypoint)
	print("\n")
	responsePub.publish(response)



def listener(dji_name = "matrice210RTK"):
	global responsePub
	scanArea = rospy.Subscriber('matrice210RTK/scanAreaCluster', ScanArea , scan_callback)
	responsePub = rospy.Publisher('matrice210RTK/ScanAreaResponse', MissionWaypointTask, queue_size=2)
	if not rospy.is_shutdown():
		rospy.spin()


		

'''
Main Function
'''
if __name__ == '__main__':
	try:
		rospy.init_node('jetsonm210RTKScanDP', anonymous=False)
		private_param = rospy.get_param('~dji_name', "matrice210RTK") 
		listener(dji_name = private_param)
	except rospy.ROSInterruptException: 
		pass
