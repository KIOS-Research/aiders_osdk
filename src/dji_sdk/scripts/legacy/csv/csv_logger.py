#!/usr/bin/env python

# Author: Andreas
# Email: anastasiou.antreas@ucy.ac.cy
# Date :26/07/2021

# license removed for brevity

import rospy
import csv
import math
from datetime import datetime
from sensor_msgs.msg import BatteryState
from dji_sdk.msg import Telemetry
from geometry_msgs.msg import Vector3Stamped


battery_state=None
telemetry_state=None
velocities_state=None

def battery_clb(data):
    global battery_state
    battery_state=data

def telemetry_clb(data):
    global telemetry_state
    telemetry_state = data

def velocities_clb(data):
    global velocities_state, telemetry_state
    yaw_rad = telemetry_state.heading * math.pi/180.0
    velocities_state = Vector3Stamped()
    velocities_state.vector.x = data.vector.x * math.cos(yaw_rad) + data.vector.y * math.sin(yaw_rad)
    velocities_state.vector.y = -1.0 * data.vector.x * math.sin(yaw_rad) + data.vector.y * math.cos(yaw_rad);
    velocities_state.vector.z = -1.0 * data.vector.z

def listener(dji_name = "matrice210v2"):
	global battery_state, telemetry_state, velocities_state
	batterySub = rospy.Subscriber(dji_name + '/battery_state', BatteryState, battery_clb)
	telemetrySub = rospy.Subscriber(dji_name + '/telemetry2', Telemetry, telemetry_clb)	
	velocitiesSub = rospy.Subscriber(dji_name + '/velocity', Vector3Stamped, velocities_clb)
	
	rateHz = 5.0
	rospy.sleep(5)

	csv_filename = "/home/jetson/catkin_ws/logs/" + datetime.now().strftime("%d%m%Y_%H%M%S") + ".csv"
	with open(csv_filename, 'w') as csvfile:
		writer = csv.writer(csvfile, delimiter=',')
		writer.writerow(['Timestamp','Battery Percentage[%]','Battery Voltage[mV]','Battery Current[mA]', 'Horizontal Velocity[m/s]', 'Altitude[m]', 'Velocity X[m/s]', 'Velocity Y[m/s]', 'Velocity Z[m/s]'])
        rospy.loginfo("csv logger started")		

	while not rospy.is_shutdown():			
		with open(csv_filename, 'a') as csvfile:
			writer = csv.writer(csvfile, delimiter=',')
			writer.writerow([battery_state.header.stamp, battery_state.percentage, battery_state.voltage, battery_state.current, telemetry_state.velocity, telemetry_state.altitude, velocities_state.vector.x, velocities_state.vector.y,velocities_state.vector.z])
		rospy.loginfo("row inserted")		
		rospy.sleep(1.0/rateHz)

'''
Main Function
'''
if __name__ == '__main__':
	try:
		rospy.init_node('jetsonm210v2csvLogger', anonymous=False)
		private_param = rospy.get_param('~dji_name', "matrice210v2") 
		listener(dji_name = private_param)
	except rospy.ROSInterruptException: 
		pass
