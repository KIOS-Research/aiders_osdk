import os, sys
import rospy

from threading import Thread, Lock

import numpy as np

from geopy import distance

from kios.msg import Telemetry
from dji_sdk.msg import BoundingBoxes

from geometry_msgs.msg import Vector3Stamped


file_path = os.path.realpath(__file__)
print(file_path)
file_path = file_path.split('scripts', 1)[0] + 'scripts/'
print(file_path)
sys.path.append(file_path)
from camera_utils import basic_camera_controls as cam_utils
#import basic_camera_controls as cam_utils



drone_telemetry = None
squad_telemetry = {}
def telemetryCB(telemetry):
	global drone_telemetry, squad_telemetry

	if telemetry.serialVersionUID == boardId:
		drone_telemetry = telemetry
	else:
		squad_telemetry[telemetry.serialVersionUID] = telemetry


client_pool_connected = []
client_sub_dict = {}
client_measurements = {}
client_pool_mutex = Lock()
def clientPoolCB(clientpool_list):
	global drone_telemetry, squad_telemetry

	if telemetry.serialVersionUID == boardId:
		drone_telemetry = telemetry
	else:
		squad_telemetry[telemetry.serialVersionUID] = telemetry


#def 

squad_telemetry_topics = []
squad_telemetry_subs = []
def squadron_telemetry_subscriber():
	global measurement_table_list, measurement_table_lock
	global fused_measurements_pub

	update_telemetry_topics_rate=rospy.Rate(0.1)

	while not rospy.is_shutdown():
		update_telemetry_topics_rate.sleep()

		topics = rospy.get_published_topics()
	
		for topic in topics:
			if boardId not in topic[0]:
				if 'Telemetry' in topic[0] and 'crps' not in topic[0]:

					if topic[0] not in squad_telemetry_topics:
						print('Detected drone in squadron:\n\tTopic:\t\t' + topic[0] + '\n\tDrone ID:\t' + topic[0].split 
	    ('_')[1].split('/')[0])
						telSub = rospy.Subscriber(topic[0], Telemetry, telemetryCB)
						squad_telemetry_topics.append(topic[0])
						squad_telemetry_subs.append(telSub)


		  

roque_telemetry = {}
def crps_detection_telemetryCB(detection_telemetry):
	pass
	#print('Track Module - Got detection')
	#if check_if_roque(detection_telemetry):
	#	roque_telemetry = detection_telemetry


def check_if_roque(detection_telemetry):
	global drone_telemetry
	print('Distance to detected drone (' + boardId + '-roque):\t' + str(geodesic_distance(detection_telemetry, drone_telemetry)))

	try:
		for squad_drone_topic in squad_telemetry_topics:
			squad_drone_id = squad_drone_topic.split('_')[1].split('/')[0]

			print('Distance to detected drone (' + squad_drone_id + '-roque):\t' + str(geodesic_distance(detection_telemetry, squad_telemetry[squad_drone_id])))
	except Exception as e:
		print('check if roque exception:', e)


def geodesic_distance(point1, point2):
	pt1 = [point1.latitude, point1.longitude, point2.altitude]
	pt2 = [point2.latitude, point2.longitude, point2.altitude]

	print('Geodesic Distance:\n\tPoint1:\t' + str(pt1) + '\n\tPoint2:\t' + str(pt2))

	distance_2d = distance.distance(pt1[:2], pt2[:2]).m
	#print('Distance 2d:\t', distance_2d)

	distance_3d = np.sqrt(distance_2d**2 + (pt2[2] - pt1[2])**2)
	#print('Distance 3d:\t', distance_3d)

	return distance_3d


g_roll = 0.0
g_pitch = 0.0
g_yaw = 0.0
def GimbalCB(gimbal):
    global g_roll, g_pitch, g_yaw
    g_roll = gimbal.vector.y
    g_pitch = gimbal.vector.x
    g_yaw = gimbal.vector.z


detector_latest_bb = None
detector_latest_bb_mutex = Lock()
def boundingBoxesCB(boundingBoxes):
	global detector_latest_bb
	detector_latest_bb_mutex.acquire()
	detector_latest_bb = boundingBoxes
	detector_latest_bb_mutex.release()


bounding_box_error_threshold = 0.05
bounding_box_step_deg = 0.5
gimbal_action_rate = 2.0
def center_drone_bounding_box():

	rate= rospy.Rate(gimbal_action_rate)
	while not rospy.is_shutdown():
		print('center_drone_bounding_box running')

		top = 0.5
		bottom = 0.5
		left = 0.5
		right = 0.5
		current_time = rospy.get_time()
		timeout_threshold = 5

		detector_latest_bb_mutex.acquire()
		if detector_latest_bb != None:
			if current_time - detector_latest_bb.header.stamp.secs < timeout_threshold:
				top = detector_latest_bb.tops[0]
				bottom = detector_latest_bb.bottoms[0]
				left = detector_latest_bb.lefts[0]
				right = detector_latest_bb.rights[0]
		detector_latest_bb_mutex.release()

		print('top:',  top)
		print('bottom:',  bottom)
		print('left:',  left)
		print('right:',  right)

		top_error = 0.5 - top
		left_error = 0.5 - left

		g_roll_initial = g_roll
		g_pitch_initial = g_pitch
		g_yaw_initial = g_yaw


		print('top_error:', top_error)
		print('left_error:', left_error)

		#print('original camera angle:', g_roll_target, g_pitch_target, g_yaw_target)
		print('original camera angle: (' + str(g_roll_initial) + ', ' +  str(g_pitch_initial) + ', ' + str(g_yaw_initial) + ')')
		g_roll_target = g_roll_initial
		g_pitch_target = g_pitch_initial
		g_yaw_target = g_yaw_initial

		call_service = False
		if abs(top_error) > bounding_box_error_threshold:
			if top_error > 0:
				g_yaw_target = g_yaw_initial - bounding_box_step_deg
			else:
				g_yaw_target = g_yaw_initial + bounding_box_step_deg

			call_service = True

		if abs(left_error) > bounding_box_error_threshold:
			if left_error > 0:
				g_pitch_target = g_pitch_initial + bounding_box_step_deg
			else:
				g_pitch_target = g_pitch_initial - bounding_box_step_deg

			call_service = True

		print('target camera angle: (' + str(g_roll_target) + ', ' +  str(g_pitch_target) + ', ' + str(g_yaw_target) + ')')
		print('offset camera angle: (' + str(g_roll_target - g_roll_initial) + ', ' +  str(g_pitch_target - g_pitch_initial) + ', ' + str(g_yaw_target - g_yaw_initial) + ')')

		if call_service:
			print('Calling angle gimbal')
			cam_utils.angle_gimbal_smoothly(g_roll_target, g_pitch_target, g_yaw_target)
			print('Calling angle gimbal DONE')

		print('=====')

		rate.sleep()




def calculate_gimbal_movement(top, bottom, left, right):
	pass



def listener(dji_name = "matrice300"):
	global boardId    
	
	dronename = dji_name
	boardId = dji_name.split('_', 1)[1]
	nodename = dronename.replace('/', '') + '_collaborative_localization_roque_tracker'

	rospy.init_node(nodename)

	rospy.Subscriber(dronename+'/gimbal_angle', Vector3Stamped, GimbalCB)
	rospy.Subscriber(dronename + '/crps/bounding_boxes', BoundingBoxes, boundingBoxesCB)


	roque_tracker_thread = Thread(target=center_drone_bounding_box)
	roque_tracker_thread.start()

	rospy.spin()



if __name__ == "__main__":
	print('Initializing Camera Roque Tracker...')
	f = open("/var/lib/dbus/machine-id", "r")
	boardId = f.read().strip()[0:4]

	dronename = 'matrice300' + '_' + boardId
	listener(dronename)

