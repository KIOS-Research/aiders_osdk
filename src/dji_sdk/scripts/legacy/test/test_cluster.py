#!/usr/bin/env python

# Author: Andreas
# Email: anastasiou.antreas@ucy.ac.cy
# Date :13/12/2019

# license removed for brevity

import rospy
import traceback
import sys
from termcolor import colored
import re
#import tf
from multiprocessing import Process
from dji_sdk.msg import ScanArea, MissionWaypoint, MissionWaypointTask
from online_cluster import Path
from std_msgs.msg import String
responsePub = None
responsePub1 = None
responsePub2 = None
votePub = None
circuitX = None
circuitY = None
cont=False
cont2=False
times=0

def vote_callback(msg):
	global cont, times
	times+=1
	if msg.data == "matrice210v2" and times==1:
		cont=True;
	print "times: ", times
	if times==1:
		rospy.sleep(6)
		times=0
		print "reset times: ", times

def scan_callback(mission):
	vote = String()
	vote.data = "matrice210v2"
	votePub.publish(vote)
	rospy.sleep(1)
	global circuitX, circuitY, responsePub, cont, cont2, times
	if cont:
		print("ResponsePub matrice210v2\n")
		cont2=True
		ll, ul, lr, ur = mission.areaCorners
		test = Path(11,11, ll, ul, lr, ur)
		test.visited([0])
		batt = mission.batteries
		circuitX, circuitY = test.calculateCircuits(batt, mission.currentPositions, mission.endingPositions)		
		response = MissionWaypointTask()
		response = mission.waypointTask
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
			print waypoint.latitude , " , ", waypoint.longitude
			response.mission_waypoint.append(waypoint)
		responsePub.publish(response)
		cont=False
		times=0
		print "reset times: ", times

def scan_callback1(mission):
	global circuitX, circuitY, responsePub1, cont2, times
	rospy.sleep(3)
	if cont2:
		print("ResponsePub matrice100\n")
		if circuitX==None and circuitY==None:
			ll, ul, lr, ur = mission.areaCorners
			test = Path(11,11, ll, ul, lr, ur)
			test.visited([0])
			batt = mission.batteries
			circuitX, circuitY = test.calculateCircuits(batt, mission.currentPositions, mission.endingPositions)	
		response = MissionWaypointTask()
		response = mission.waypointTask
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
			print waypoint.latitude , " , ", waypoint.longitude
			response.mission_waypoint.append(waypoint)
		responsePub1.publish(response)
		cont2=False
		times=0
		print "reset times: ", times

def scan_callback2(mission):
	global circuitX, circuitY, responsePub2, cont2, times
	rospy.sleep(3)
	if cont2:
		print("ResponsePub matrice210\n")
		response = MissionWaypointTask()
		response = mission.waypointTask
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
			print waypoint.latitude , " , ", waypoint.longitude
			response.mission_waypoint.append(waypoint)
		responsePub2.publish(response)
		cont2=False
		times=0
		print "reset times: ", times


def listener(dji_name = "matrice210v2"):
	global responsePub, responsePub1, responsePub2, votePub
	votePub = rospy.Publisher('swarm/Vote', String, queue_size=2)
	voteSub = rospy.Subscriber('swarm/Vote', String , vote_callback)
	scanArea3 = rospy.Subscriber('matrice210/scanAreaCluster', ScanArea , scan_callback2)
	scanArea = rospy.Subscriber('matrice210v2/scanAreaCluster', ScanArea , scan_callback)
	scanArea2 = rospy.Subscriber('matrice100/scanAreaCluster', ScanArea , scan_callback1)
	responsePub2 = rospy.Publisher('matrice210/ScanAreaResponse', MissionWaypointTask, queue_size=2)
	responsePub = rospy.Publisher('matrice210v2/ScanAreaResponse', MissionWaypointTask, queue_size=2)
	responsePub1 = rospy.Publisher('matrice100/ScanAreaResponse', MissionWaypointTask, queue_size=2)
	rospy.spin()
	

		

'''
Main Function
'''
if __name__ == '__main__':
	try:
		rospy.init_node('jetsonm210v2ScanCluster', anonymous=False)
		private_param = rospy.get_param('~dji_name', "matrice210v2") 
		listener(dji_name = private_param)
	except rospy.ROSInterruptException: 
		pass
