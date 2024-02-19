#!/usr/bin/env python

# Author: Andreas
# Email: anastasiou.antreas@ucy.ac.cy
# Date :15/07/2019

# license removed for brevity

import rospy
import traceback
import sys
from termcolor import colored
import re
#import tf
from multiprocessing import Process
from dji_sdk.msg import ScanArea, MissionWaypoint, MissionWaypointTask, Telemetry
from dji_sdk.srv import MissionWpAction, DroneTaskControl, ObtainControlAuthority
from online import Path
from std_msgs.msg import String, Int8
from sensor_msgs.msg import Joy
import numpy as np
import time

responsePub = None
tempPub = None
translator = None

modePub = None
tracking = 1
searching = 0
inactive = -1
RTKmode = inactive
m210mode = inactive
m100mode = inactive
flightCtrl = None
m210ScanAreaPub = None
m100ScanAreaPub = None
m210Action = None
m100Action = None
m210Mission = None
m100Mission = None
nextVisit = []
lat = None
lon = None

def RTKmodeCallback(msg):
	global RTKmode, tracking, searching, inactive, m210mode, m100mode, m210ScanAreaPub, m100ScanAreaPub, m210Action, m100Action, m210Mission, m100Mission
	if msg.data==tracking:
		RTKmode = tracking
		if m210mode==searching and m100mode==searching:
	                try:
                            result = m210Action(1)
                            result2 = m100Action(1)
                        except rospy.ServiceException, e:
                            print "Service Call Failed: %s"%e
                        scanArea = ScanArea()
			scanArea.header = m210Mission.header
			scanArea.agentNumber = 0
			scanArea.batteries = [1000, 1000]
			scanArea.altitude = m210Mission.altitude
			scanArea.areaCorners = m210Mission.areaCorners
			scanArea.endingPositions = m210Mission.endingPositions
			scanArea.waypointTask = m210Mission.waypointTask
			m210ScanAreaPub.publish(scanArea)
			scanArea.agentNumber = 1
                        scanArea.altitude = m100Mission.altitude
			m100ScanAreaPub.publish(scanArea)
                        try:
                            rospy.sleep(3)
                            result = m210Action(0)
                            result2 = m100Action(0)
                        except rospy.ServiceException, e:
                            print "Service Call Failed: %s"%e
		elif  m210mode==searching and (m100mode==tracking or m100mode==inactive):
                        try:
                            result = m210Action(1)
                        except rospy.ServiceException, e:
                            print "Service Call Failed: %s"%e
                        scanArea = ScanArea()
			scanArea.header = m210Mission.header
			scanArea.agentNumber = 0
			scanArea.batteries = [1000]
			scanArea.altitude = m210Mission.altitude
			scanArea.areaCorners = m210Mission.areaCorners
			scanArea.endingPositions = m210Mission.endingPositions
			scanArea.waypointTask = m210Mission.waypointTask
			m210ScanAreaPub.publish(scanArea)
                        try:
                            rospy.sleep(3)
                            result = m210Action(0)
                            print result
                        except rospy.ServiceException, e:
                            print "Service Call Failed: %s"%e
		elif  (m210mode==tracking or m210mode==inactive) and m100mode==searching:
			try:
                            result2 = m100Action(1)
                        except rospy.ServiceException, e:
                            print "Service Call Failed: %s"%e
                        scanArea = ScanArea()
			scanArea.header = m100Mission.header
			scanArea.agentNumber = 0
			scanArea.batteries = [1000]
			scanArea.altitude = m100Mission.altitude
			scanArea.areaCorners = m100Mission.areaCorners
			scanArea.endingPositions = m100Mission.endingPositions
			scanArea.waypointTask = m100Mission.waypointTask
			m100ScanAreaPub.publish(scanArea)
                        try:
                            rospy.sleep(3)
                            result2 = m100Action(0)
                        except rospy.ServiceException, e:
                            print "Service Call Failed: %s"%e
	elif msg.data==searching:
		RTKmode = searching
	elif msg.data==inactive:
		RTKmode = inactive

def m100modeCallback(msg):
	global m100mode, tracking, searching, inactive
	if msg.data==tracking:
		m100mode = tracking
	elif msg.data==searching:
		m100mode = searching
	elif msg.data==inactive:
		m100mode = inactive

def m210modeCallback(msg):
	global m210mode, tracking, searching, inactive
	if msg.data==tracking:
		m210mode = tracking
	elif msg.data==searching:
		m210mode = searching
	elif msg.data==inactive:
		m210mode = inactive

def scan_callback(mission):
	global mission1, lat, lon, flightCtrl

	try:
		obtnAuth = rospy.ServiceProxy('matrice210v2/obtain_release_control_authority', ObtainControlAuthority)
		resp = obtnAuth(True)
		print('Obtain Authority: ',resp.result)
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e
	mission1 = mission
	ll, ul, lr, ur = mission.areaCorners
	test = Path(10, 10, ll, ul, lr, ur)
	initTranslator(mission, test)
	test.visited([0])
	batt = mission.batteries
	visitedNodes = [0 for i in range (0,100)]
        visitedNodes[0] = 1
        pos = [ll.latitude, ll.longitude]
	circuitX, circuitY, vis = test.calculateCircuits(batt, [pos, pos], [pos, pos], mission.waypointTask.idle_velocity, visitedNodes)
	global responsePub, RTKmode, tracking, searching, nextVisit
	newMode = Int8()
	newMode.data = searching
	modePub.publish(newMode)
	mission.waypointTask.gimbal_pitch_mode = 1
	#mission.waypointTask.yaw_mode = 3
	response = MissionWaypointTask()
	response = mission.waypointTask
        response.mission_waypoint = []
        nextVisit = []
	for i in range(0,len(circuitX[mission.agentNumber])-1):
		#print circuitX[mission.agentNumber][i] , "," , circuitY[mission.agentNumber][i]
		nextVisit.append(vis[mission.agentNumber][i])
		waypoint = MissionWaypoint()		
		waypoint.latitude = circuitX[mission.agentNumber][i]
		waypoint.longitude = circuitY[mission.agentNumber][i]
		waypoint.altitude = mission.altitude
		waypoint.damping_distance = 4
		waypoint.target_yaw = 0
		waypoint.target_gimbal_pitch = -900
		waypoint.turn_mode = 0
		waypoint.has_action = 1
		waypoint.action_time_limit = 100
		waypoint.waypoint_action.action_repeat = 1 #total running times 1 (upper 4 bits) => 16 + total number of actions (lower 4 bits) => 2 ---> total = 16 + 2 = 18
		waypoint.waypoint_action.command_list = [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
		waypoint.waypoint_action.command_parameter = [0,500,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
		#print waypoint.latitude , " , ", waypoint.longitude
		response.mission_waypoint.append(waypoint)
	print("\n")
	responsePub.publish(response)

def initTranslator(mission, test):
	transInit = MissionWaypointTask()
	transInit = mission.waypointTask
	waypoint1 = MissionWaypoint()
	waypoint1.latitude = test.X[0]
	waypoint1.longitude = test.Y[0]
	waypoint1.altitude = mission.altitude
	transInit.mission_waypoint.append(waypoint1)
	waypoint2 = MissionWaypoint()
	waypoint2.latitude = test.X[10]
	waypoint2.longitude = test.Y[10]
	transInit.mission_waypoint.append(waypoint2)
	global translator
	translator.publish(transInit)

def m210Scan_callback(msg):
    global m210Mission
    m210Mission = msg

def m100Scan_callback(msg):
    global m100Mission
    m100Mission = msg

def waypoint_callback(msg):
	global nextVisit, nodes
	if ('WP Mission:' in msg.data):
		#set each node's visit potential to 0 once it's visited by the agent
		if len(nextVisit)>0:
			first = msg.data.split(' ')
			second = first[2].split("/")
			index = int(second[0])
			currentPosition = nextVisit[index]
			
		 	
def telemetryCallback(msg):
	global lat, lon
	lat = msg.latitude
	lon = msg.longitude
	
def listener(dji_name = "matrice210v2"):
	global translator, searching, tracking, modePub, tempPub, responsePub, m210ScanAreaPub, m100ScanAreaPub, m210Action, m100Action, flightCtrl
	scanArea = rospy.Subscriber('matrice210v2/scanArea', ScanArea , scan_callback)
	waypointCallback = rospy.Subscriber('matrice210v2/mission_waypoint_report', String , waypoint_callback)
	telemetrySub = rospy.Subscriber('matrice210v2/Telemetry', Telemetry , telemetryCallback)
	#flightCtrl = rospy.Publisher('/matrice210v2/flight_control_setpoint_generic', Joy, queue_size=10)
        m210scanArea = rospy.Subscriber('matrice210/scanArea', ScanArea , m210Scan_callback)
        m100scanArea = rospy.Subscriber('matrice100/scanArea', ScanArea , m100Scan_callback)
	m210ScanAreaPub = rospy.Publisher('matrice210/scanArea', ScanArea, queue_size=2)
	m100ScanAreaPub = rospy.Publisher('matrice100/scanArea', ScanArea, queue_size=2)
	responsePub = rospy.Publisher('matrice210v2/ScanAreaResponse', MissionWaypointTask, queue_size=2)
	tempPub = rospy.Publisher('matrice210v2/CPUTemperature', String, queue_size=10)
	translator = rospy.Publisher('matrice210v2/translatorInit', MissionWaypointTask, queue_size=2)
	modePub = rospy.Publisher('matrice210v2/modeChange', Int8, queue_size=10)
	modeSubRTK = rospy.Subscriber('matrice210v2/modeChange', Int8, RTKmodeCallback)
	modeSub210 = rospy.Subscriber('matrice210/modeChange', Int8, m210modeCallback)
	modeSub100 = rospy.Subscriber('matrice100/modeChange', Int8, m100modeCallback)
        m210Action = rospy.ServiceProxy('/matrice210/mission_waypoint_action', MissionWpAction)
        m100Action = rospy.ServiceProxy('/matrice100/mission_waypoint_action', MissionWpAction)
	while not rospy.is_shutdown():
		f= open("/sys/devices/virtual/thermal/thermal_zone0/temp", "r") 			
		if f.mode == "r":
			temp = f.read()
			tempResponse = String()
			tempResponse.data = temp
			tempPub.publish(tempResponse)
		rospy.sleep(1)


		

'''
Main Function
'''
if __name__ == '__main__':
	try:
		rospy.init_node('jetsonm210v2Scan', anonymous=False)
		private_param = rospy.get_param('~dji_name', "matrice210v2") 
		listener(dji_name = private_param)
	except rospy.ROSInterruptException: 
		pass
