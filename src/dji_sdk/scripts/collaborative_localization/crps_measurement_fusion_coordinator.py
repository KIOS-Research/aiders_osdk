# Author: Christos Georgiades
# Date: 2023-02-20

import rospy

import os, sys
import time


from threading import Thread, Lock
import atexit

from std_msgs.msg import String, Bool
from kios.msg import Telemetry
from raft.msg import AppendEntriesRPC

import copy

import pandas as pd
import datetime
import subprocess

ros_version = subprocess.check_output('rosversion -d', shell=True).decode('utf-8').strip()
print('ros_version:', ros_version)
if ros_version == 'melodic':
	print('multimaster_msgs_fkie')
	from multimaster_msgs_fkie.msg import LinkStatesStamped
elif ros_version == 'noetic':
	print('fkie_multimaster_msgs')
	from fkie_multimaster_msgs.msg import LinkStatesStamped

import Monitoring as monitoring

sys.path.append('/home/jetson/Documents/aiders_osdk/src/dji_sdk/scripts/logging_system')
import logging_commons as log

measurements_topic = '/crps/Telemetry'
coordinator_topic = '/crps/coordinator'
coordinator_name_topic = '/crps/coordinator_dronename'
current_coordinator=None


client_pool_connected = []
client_sub_dict = {}
client_measurements = {}
client_measurements_time = {}
client_pool_mutex = Lock()
def clientPoolCB(client_pool_msg):
	global client_pool_connected, client_sub_dict

	client_pool_mutex.acquire()
	client_pool_prev = copy.deepcopy(client_pool_connected)
	#client_sub_dict_prev = copy.deepcopy(client_sub_dict)
	client_pool_mutex.release()

	client_pool_new = parse_client_pool(client_pool_msg)
	client_pool_add = list(set(client_pool_new) - set(client_pool_prev))
	client_pool_sub = list(set(client_pool_prev) - set(client_pool_new))

	#print(client_pool_new)
	#print(client_pool_add)
	#print(client_pool_sub)

	for new_client in client_pool_add:
		print('Subscribed to:', '/matrice300_' + new_client + measurements_topic)
		crpsSub = rospy.Subscriber('/matrice300_' + new_client + measurements_topic, Telemetry, crpsMeasurementCB)
		client_sub_dict[new_client] = crpsSub

	for old_client in client_pool_sub:
		print('Unsubscribed to:', '/matrice300_' + old_client + measurements_topic)
		client_sub_dict[old_client].unregister()
		del(client_sub_dict[old_client])

	client_pool_mutex.acquire()
	client_pool_connected = client_pool_new
	client_pool_mutex.release()


def parse_client_pool(client_pool_msg):
	client_pool_txt = client_pool_msg.data
	client_pool_split = client_pool_txt.split("': ")
	client_pool_list = []

	for i in range(len(client_pool_split)-1):
		client_pool_list.append(client_pool_split[i].split("'")[1])

	#print('client_pool_txt:', client_pool_txt)
	#print('client_pool_list:', client_pool_list)

	return client_pool_list


crps_measurement_dict = {}
crps_measurement_dict_mutex = Lock()
def crpsMeasurementCB(crpsMeasurement):
	droneId = crpsMeasurement.serialVersionUID.split('_')[1].split('-')[0]
	#print('Got measurement. DroneId:', droneId)

	crps_measurement_dict_mutex.acquire()
	crps_measurement_dict[droneId] = crpsMeasurement
	crps_measurement_dict_mutex.release()


squad_leader = None
def raftLeaderCB(raftLeader):
	global squad_leader
	#print('Got raft leader:', raftLeader)
	squad_leader = raftLeader.leaderId


fused_measurements_value = Telemetry()
def fuse_measurements():
	global measurement_dict, measurement_dict_lock
	global crps_measurement_dict, crps_measurement_dict_mutex
	global fused_measurements_value, fused_measurements_pub
	global squad_leader
	global crps_state_enabled

	measurement_fuse_rate=rospy.Rate(10)

	print('fuse_measurements thread started')
	while crps_state_enabled and not rospy.is_shutdown():
		crps_measurement_dict_mutex.acquire()	
		latest_crps_measurements_dict = copy.deepcopy(crps_measurement_dict)
		crps_measurement_dict = {}
		crps_measurement_dict_mutex.release()
		#print('Deep Copy successful')
		
		latest_crps_measurements_list = list(latest_crps_measurements_dict.values())

		#print('cl - Fusing...')
		fused_measurements = merge_telemetry(latest_crps_measurements_list)
		log_measurements(latest_crps_measurements_list, fused_measurements)
		#print('cl - Logging...\n', str(fused_measurements))

		if fused_measurements:
			#print('Measurements Fused (lon, lat):', '(' + str(fused_measurements.longitude) + ', ' + str(fused_measurements.latitude) + ')')

			print(boardId, squad_leader)
			if boardId == squad_leader or True:
				print(boardId, '- Published fused_measurements')
				fused_measurements_pub.publish(fused_measurements)
		
		fused_measurements_value = fused_measurements

		#measurement_dict_lock.acquire()
		#measurements_list = measurement_dict
		#measurement_dict_lock.release()
		#print('Publish successful')

		measurement_fuse_rate.sleep()

		# if current_coordinator == ip and False:	
		# 	if len(measurements_list) > 0 and False:
		# 		weight =  1 / len(measurements_list)

		# 		heading = 0.0
		# 		longitude = 0.0
		# 		latitude = 0.0

		# 		#print('measurements_list:', measurements_list)
		# 		try:
		# 			for drone_id in measurements_list:
		# 				measurement = measurements_list[drone_id]
		# 				heading = heading + weight * measurement.heading
		# 				longitude = longitude + weight * measurement.longitude
		# 				latitude = latitude + weight * measurement.latitude

					

		# 			fused_measurents = Telemetry()
		# 			fused_measurents.rostime_secs = int(time.time())
		# 			fused_measurents.rostime_nsecs = int(time.time() % 1 * 10**9)

		# 			fused_measurents.latitude = latitude
		# 			fused_measurents.longitude = longitude
		# 			fused_measurents.heading = heading
		# 			fused_measurents.serialVersionUID = boardId + '_fused'
		# 			fused_measurements_pub.publish(fused_measurents)
		# 			fused_measurements_value = fused_measurents
		# 			print('Fusion - ('+ dronename + '-' + ip +') - Fused Measurements Table')
		# 		except Exception as e:
		# 			print('fuse_measurements - Caught Exception:', e)
		# 			#print('heading:', heading)
		# 			#print('longitude:', longitude)
		# 			#print('latitude:', latitude)
		# 			#print('measurements_list:', measurements_list)
		# 			#print('weight:', weight)
		# 			pass


crps_timeout = 30		
def remove_invalid_measurements(telemetry_list):
	current_time = time.time()
	valid_telemetry_list = []

	for i in range(len(telemetry_list)):
		time_diff = current_time - telemetry_list[i].rostime_secs

		if time_diff < crps_timeout:
			valid_telemetry_list.append(telemetry_list[i])

	return valid_telemetry_list


def merge_telemetry(telemetry_list): 
	valid_telemetry_list = remove_invalid_measurements(telemetry_list)

	#print('valid_telemetry_list size:', len(valid_telemetry_list))

	if len(valid_telemetry_list) > 0:
		weight = len(valid_telemetry_list)

		altitude = 0.0
		heading = 0.0
		longitude = 0.0
		latitude = 0.0

		#print('fuse_telemetry weight:', weight)
		try:
			print('Merge Telemetry Breakdown:')
			for measurement in valid_telemetry_list:
				altitude = altitude + measurement.altitude		
				heading = heading + measurement.heading
				longitude = longitude + measurement.longitude
				latitude = latitude + measurement.latitude

				print(measurement.serialVersionUID +  ':\n\t' + str(measurement.altitude) + ':\n\t' + str(measurement.heading) + '\n\t' + str(measurement.longitude) + '\n\t' + str(measurement.latitude))
			
			heading = heading / weight
			longitude = longitude / weight
			latitude = latitude / weight

			print('\nWeight:\t', weight)
			print('Final Values:', heading, longitude, latitude)

			fused_measurents = Telemetry()
			fused_measurents.rostime_secs = int(time.time())
			fused_measurents.rostime_nsecs = int(time.time() % 1 * 10**9)

			fused_measurents.altitude = altitude
			fused_measurents.latitude = latitude
			fused_measurents.longitude = longitude
			fused_measurents.heading = heading
			fused_measurents.serialVersionUID = boardId + '_fused'
			return fused_measurents
			#fused_measurements_pub.publish(fused_measurents)
			#fused_measurements_value = fused_measurents
			print('Fusion - ('+ dronename + '-' + ip +') - Fused Measurements Table')
		except Exception as e:
			print('fuse_measurements - Caught Exception:', e)

	return None


initTime = str(datetime.datetime.now().strftime("%Y-%m-%d-%H:%M"))
def log_measurements(latest_crps_measurements_list, fused_measurements):
	global measurement_dict, measurement_dict_lock, fused_measurements_value
	global initTime

	measurements_table = latest_crps_measurements_list
	fused_measurements_value = fused_measurements

	#print('measurements_table size:', len(measurements_table))
	#print('measurements_table:', measurements_table)
	#if len(measurements_table) > 0:
	#	print('measurements_table[0]:', measurements_table[0])
	#print('fused_measurements_value:', fused_measurements_value)

	for tele in measurements_table:
		#tele = measurements_table[key]
		if tele is not None:
			drone_id = tele.serialVersionUID.split('_')[0]
			
			file1 = open(flightlog_path + drone_id + "_measurement-" + initTime +".csv", "a")  # append mode
			file1.write(f"{str(datetime.datetime.now())},{tele.longitude},{tele.latitude}\n")
			file1.close()

			print('Saved ' + drone_id + ' measurement')
	
	if fused_measurements_value is not None:
		tele = fused_measurements_value
		file1 = open(flightlog_path + "fused_measurement-" + initTime + ".csv", "a")  # append mode
		file1.write(f"{str(datetime.datetime.now())},{tele.longitude},{tele.latitude}\n")
		file1.close()

		print('Saved fused measurement')
	
	#measurement_log_rate.sleep()


def set_crps_state_CB(state_enable):
	global crps_state_enabled, crps_state_pub

	state_enable = state_enable.data

	print('set_module_state_CB - got state_enable:', state_enable)
	if state_enable: 
		if not crps_state_enabled:
			crps_state_enabled = True

			fuse_measurements_thread = Thread(target=fuse_measurements)
			fuse_measurements_thread.start()

			monitoring.set_monitoring_state(state_enable)
	else:
		crps_state_enabled = False
		monitoring.set_monitoring_state(state_enable)

	
	crps_state_pub.publish(crps_state_enabled)
	#log.create_publish_log('Started CRPS thread', 'Drone in squadron requested CRPS')


flightlog_path = ''
def flight_log_CB(flightlog_path_pckt):
	global flightlog_path

	flightlog_path = flightlog_path_pckt.data
	print('Fusion Coordinator - flightlog_path:', flightlog_path)


def init():
	global crps_state_enabled, crps_state_pub
	global flightlog_path
	global fused_measurements_pub

	crps_state_enabled = False

	#rospy.Subscriber('/master_discovery/linkstats', LinkStatesStamped, availableDronesCB)
	rospy.Subscriber(dronename+'/crps/Request', Bool, set_crps_state_CB)
	crps_state_pub = rospy.Publisher(dronename+'/crps/State', Bool, queue_size=1, latch=True)
	crps_state_pub.publish(crps_state_enabled)


	rospy.Subscriber('/raft/client_pool', String, clientPoolCB)
	rospy.Subscriber('/raft/raft_leader', AppendEntriesRPC, raftLeaderCB)

	rospy.Subscriber(dronename + '/FlightLogDirectory', String, flight_log_CB)
	
	#coordinator_pub = rospy.Publisher(coordinator_topic, String, queue_size=1, latch=True)
	#coordinator_name_pub = rospy.Publisher(coordinator_name_topic, String, queue_size=1, latch=True)

	fused_measurements_pub = rospy.Publisher(measurements_topic, Telemetry, queue_size=1, latch=True)


	#update_coordinator_thread = Thread(target=update_coordinator)
	#read_measurements_thread = Thread(target=read_measurements)
	
	#fuse_measurements_thread = Thread(target=fuse_measurements)
	#fuse_measurements_thread.start()

	#log_measurements_thread = Thread(target=log_measurements)

	#update_coordinator_thread.start()
	#read_measurements_thread.start()
	
	#log_measurements_thread.start()

	monitoring.init()
	log.init()

	#print('collaborative_localization_coordinator started threads')

	# atexit.register(disconnect_coordinator)



def listener(dji_name = "matrice300"):
	global dronename, boardId
	global coordinator_pub, coordinator_name_pub, fused_measurements_pub
	global flightlog_path


	dronename = dji_name
	boardId = dji_name.split('_', 1)[1]
	nodename = dronename.replace('/', '') + '_collaborative_localization_coordinator'

	rospy.init_node(nodename)

	init()

	rospy.spin()


if __name__ == '__main__':
	f = open("/var/lib/dbus/machine-id", "r")
	boardId = f.read().strip()[0:4]
	
	file_path = os.path.realpath(__file__)
	file_path = file_path.split('src', 1)[0] + 'launch/'

	print(file_path)
	sys.path.insert(1, file_path)

	import getActiveIP
	ip = getActiveIP.getActiveIp()

	print('Collaborative Localization Coordinator(' + boardId + '-' + ip + ') starting...')
	
	dronename = 'matrice300' + '_' + boardId
	listener(dronename)
