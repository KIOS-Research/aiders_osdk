#!/usr/bin/env python

# Author: Christos Georgiades
# Email: cgeorg15@ucy.ac.cy
# Date : 02/08/2023

# Rudimentary fault detection for Sesame 2023

import rospy

import sys

import threading
from math import sin, cos, atan2, sqrt

from dji_sdk.msg import ErrorLog, ErrorLogList

from dji_sdk.msg import MissionWaypoint
from dji_sdk.msg import JoystickParams, MissionWaypointTask
from dji_sdk.msg import FlightAnomaly


from std_msgs.msg import Bool, String, Int16
from dji_sdk.srv import ObtainControlAuthority

from kios.msg import Telemetry

from drone_mission import DroneMission

sys.path.append('/home/jetson/Documents/aiders_osdk/src/dji_sdk/scripts/logging_system')
import logging_commons as log

fault_dictionary = {}


altitude = 0.0
longitude = 0.0
latitude = 0.0
yaw = 0.0

battery_percentage = 0.0
battery_threshold = 0.0

gps_signal = 0.0
satellite_number = 0.0
def telemetryCB(tele):
    global sat_num_Pub

    global altitude, longitude, latitude, yaw
    global battery_percentage, battery_threshold
    global gps_signal, satellite_number

    altitude = tele.altitude
    longitude = tele.longitude
    latitude = tele.latitude
    yaw = tele.heading

    battery_percentage = tele.batteryPercentage
    battery_threshold = tele.batteryThreashold
    
    gps_signal = tele.gpsSignal
    satellite_number = tele.satelliteNumber

    #sat_num_Pub.publish(satellite_number)


wind_level = 0
STRONG_WIND_LEVEL1         = 64    #
STRONG_WIND_LEVEL2         = 128   #
def flight_anomaly_CB(anomaly):
    global wind_level
    global wind_level_Pub

    anomaly_data = anomaly.data

    if anomaly_data == STRONG_WIND_LEVEL1:
        wind_level = 1
    elif anomaly_data == STRONG_WIND_LEVEL2:
        wind_level = 2
    else:
        wind_level = 0

    # if wind_level == 0:
    #     wind_level_Pub.publish(False)
    # else:
    #     wind_level_Pub.publish(True)


sat_number_min_threshold = 8
gps_signal_lost = False
def gps_fault_detection():
    global battery_percentage, battery_threshold
    global gps_signal, satellite_number
    global gps_signal_lost


    error_log_list = []

    #print(satellite_number, sat_number_min_threshold, gps_signal_lost)
    #if satellite_number < 

    if satellite_number < sat_number_min_threshold:
        if not gps_signal_lost:
            gps_signal_lost = True

            message = 'Aircraft lost GPS signal.'
            context = 'satellite_number < ' + str(sat_number_min_threshold)

            error_log_list.append(log.create_log(message, context, 100, 1))
    else:    
        if gps_signal_lost:
            gps_signal_lost = False

            message = 'Aircraft regained GPS signal.'
            context = 'satellite_number > ' + str(sat_number_min_threshold)
            error_log_list.append(log.create_log(message, context, 100, 0))
            
    return gps_signal_lost, error_log_list


error_dict = {}
def update_error(error_log_list):
    global error_dict
    global FaultDetectionPub

    #print('error_log_list_size:', len(error_log_list))
    #print('error_dict:', len(error_dict))
    for error_log in error_log_list:
        error_id = error_log.error_id

        #print('error_id:', error_id, 'error_log.severity:', error_log.severity)
         
        if error_log.severity > 0:
            if not error_id in error_dict:
                FaultDetectionPub.publish(error_log)
            
            error_dict[error_id] = error_log
        else:
            if error_id in error_dict:
                del error_dict[error_id]
                FaultDetectionPub.publish(error_log)

    return error_log_list


            


def publish_detected_faults(error_dict, publisher):
    for key in error_dict.keys():
        print(key, error_dict[key])
        error_log = error_dict[key]
        publisher.publish(error_log)


def get_fault_dict():
    return error_dict


def fault_detection_worker():    
    rate = rospy.Rate(0.2)
    rate.sleep()

    while not rospy.is_shutdown():
        #gps_fault, error_log_list = gps_fault_detection()
        #update_error(error_log_list)
        #publish_detected_faults(error_dict, FaultDetectionPub)

        #if len(error_log_list):
            #fault_list_string = ""
            #for error_log in error_log_list:
            #    fault_list_string = fault_list_string + log.log_to_string(error_log) + '\n'

            #FaultListPub.publish(fault_list_string)

        rate.sleep()


def init():
    global FaultDetectionPub, FaultListPub
    global wind_level_Pub, sat_num_Pub

    if rospy.is_shutdown():
        print(nodename,'- init()','- rospy is not ready')
    else:
        FaultDetectionPub = rospy.Publisher(dronename+'/Log', ErrorLog, queue_size=10)
        #FaultListPub = rospy.Publisher(dronename+'/Log/List', String, queue_size=1, latch=True)

        sat_num_Pub = rospy.Publisher(dronename+'/kios/accuracy', Int16, queue_size=1)
        
        rospy.Subscriber(dronename+'/Telemetry', Telemetry, telemetryCB)

        rospy.Subscriber(dronename+'/flight_anomaly', FlightAnomaly, flight_anomaly_CB)
        wind_level_Pub = rospy.Publisher(dronename+'/kios/wind', Bool, queue_size=1)

        


        log.init()

        fault_detection_worker_thread = threading.Thread(target=fault_detection_worker)
        fault_detection_worker_thread.start()

        print(nodename,'- init()','- DONE')


def listener(dji_name = "/matrice300", as_module = False):
    global dronename, nodename

    dronename = dji_name
    nodename = dronename.replace('/', '') + '_fault_detection'
    #print(nodename)

    if not as_module:
        rospy.init_node(nodename)
        init()
    else:
        print(nodename,'- Imported as a module. Call init when ready to initialize rostopics...')

    if not as_module:
        rospy.spin()


print('Initializing Fault Detection...')
f = open("/var/lib/dbus/machine-id", "r")
boardId = f.read().strip()[0:4]

dronename = '/matrice300' + '_' + boardId

if __name__ == '__main__':
    as_module = False
else:
    as_module = True

listener(dji_name=dronename, as_module=as_module)
