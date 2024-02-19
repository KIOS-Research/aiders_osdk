#!/usr/bin/env python

import sys
from dji_sdk.msg import telemetry2
from kios.msg import Telemetry, Positioning
from std_msgs.msg import String, Int8, Bool

from sensor_msgs.msg import BatteryState

from geometry_msgs.msg import Vector3Stamped, QuaternionStamped

from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
rad2deg = 57.2958
deg2rad = 0.0174533

import rospy
import threading

import positioning_system_priority as psp

boardId = ""
translator= None

home_latitude = 0.0
home_longitude = 0.0

droneState='Landed'
# In_Mission
# Paused_Mission
# Flying
# Going_Home
# Armed
# Landed

seq = 0

safe_satellite_no = 8


# Drone Quaternion
d_quat_x = 0.0
d_quat_y = 0.0
d_quat_z = 0.0
d_quat_w = 0.0

# Drone Quaternion
d_roll = 0.0
d_pitch = 0.0
d_yaw = 0.0
def DroneOrientationCB(msg):
    global d_quat_x, d_quat_y, d_quat_z, d_quat_w
    global d_roll, d_pitch, d_yaw
    
    d_quat_x = msg.quaternion.x
    d_quat_y = msg.quaternion.y
    d_quat_z = msg.quaternion.z
    d_quat_w = msg.quaternion.w 
    
    quat = [d_quat_x, d_quat_y, d_quat_z, d_quat_w]
    (d_roll, d_pitch, d_yaw) = euler_from_quaternion(quat)
    d_roll *= rad2deg
    d_pitch *= rad2deg * -1.0
    d_yaw *= rad2deg 


# Gimbal
g_yaw = 0
g_pitch = 0
g_roll = 0
def GimbalOrientationCB(gimbal):
    global g_yaw, g_pitch, g_roll
    g_yaw = gimbal.vector.z
    g_pitch = gimbal.vector.x
    g_roll = gimbal.vector.y


def DroneStateCB(state):
    global droneState
    droneState = str(state.data)
    #print(state)


#false_battery_failure = False
batteryPercentage = 0
batteryVoltage = 0
batteryCurrent = 0
batteryCapacity = 0
def batteryCB(batteryStatus):
    global batteryPercentage, batteryVoltage, batteryCurrent, batteryCapacity
    global false_battery_failure

    batteryPercentage = batteryStatus.percentage
    batteryVoltage = batteryStatus.voltage
    batteryCurrent = batteryStatus.current
    batteryCapacity = batteryStatus.capacity

    #if false_battery_metric > 0:
    #    batteryPercentage = false_battery_metric


false_battery_metric = 0
def battery_failure_CB(battery_failure_state):
    global false_battery_metric
    false_battery_metric = battery_failure_state.data
    print('\nfalse_battery_metric:', false_battery_metric, '\n')


false_gps_metric = 0
def gps_failure_CB(gps_failure_state):
    global false_gps_metric
    false_gps_metric = gps_failure_state.data
    print('false_gps_metric:', false_gps_metric)

# altitudeLandedLimit = 2.0
# def calcDroneState(altitude):
#     global droneState
    
#     #print ('In dronestate', dronestate)
    
#     if droneState == 'Flying' or droneState == 'Landed':
#         if altitude > altitudeLandedLimit:
#             droneState = 'Flying'
#         else:
#             droneState = 'Landed'
            
#     return droneState


# gpsTelemetry = None
# def telemetryCB(tele):
#     global gpsTelemetry
#     gpsTelemetry = tele

#     if not homePositionSet:
#         set_home_position(tele)

#     publishTelemetry(tele, translator)
#     publishPositioning(tele)


# fusedTelemetry = None
# def fusedTelemetryCB(tele):
#     global fusedTelemetry, boardId

#     if boardId in tele.serialVersionUID:
#         fusedTelemetry = tele
#         print('Got fusedTelemetry - Packet title:\t', tele.serialVersionUID)


# fusedTelemetryTimeoutMin = 2
# def get_valid_telemetry():
#     global gpsTelemetry, fusedTelemetry

#     gpsTelePckt = gpsTelemetry
#     fusedTelePckt = fusedTelemetry

#     now = rospy.Time.now()  

#     if fusedTelemetry is not None:
#         if tele.satelliteNumber < 8:
#             minutes_diff = (now.secs - fusedTelemetry.rostime_secs) / 60.0
#             if minutes_diff > fusedTelemetryTimeoutMin:
#                 fusedTelemetry = None
#             if packet_age != minutes_diff:
#                 print('Using Fused Telemetry - Packet age:\t', minutes_diff)

#             packet_age = minutes_diff
#             tele = fusedTelemetry


#     # Check if gpsTelemetry is valid
#     # if tele.satelliteNumber < 8:
#     #     # Fallback to fused telemetry if available       
#     #     if fusedTelemetry is not None:
#     #         


homePositionSet = False
homePositionConf = 0.0
homePositionConf_Limit = 0.8
homePositionConf_window = 25
home_latitude = 0.0
home_longitude = 0.0
initial_timestamp = 0.0
def set_home_position(tele):
    global homePositionSet, home_latitude, home_longitude, initial_timestamp
    global homePositionConf, homePositionConf_Limit, homePositionConf_window
    global homePositionPub

    if not homePositionSet:
        if tele.satelliteNumber > safe_satellite_no:
            if homePositionConf < homePositionConf_Limit:
                homePositionConf = homePositionConf + 1 / homePositionConf_window
                #print('homePositionConf:', homePositionConf)
            else:         
                home_latitude = tele.latitude
                home_longitude = tele.longitude 
                homePositionSet = True

                tele.homeLatitude = home_latitude
                tele.homeLongitude = home_longitude

                tele.serialVersionUID = dronename

                homePositionPub.publish(tele)
                print(dronename, '- Home Position Set\n', '\tlat:', home_latitude, 'lon:', home_longitude, '\n\tinitial_timestamp:', initial_timestamp)


packet_age = 0.0
def publishTelemetry(tele, publisher):
    global droneState, altitude, seq
    global gpsTelemetry, fusedTelemetry
    global packet_age

    global false_gps_metric
    global batteryPercentage
    
    now = rospy.Time.now()
    telePckt = Telemetry()
    
    #telePckt.seq = seq
    #telePckt.uid = boardId
    if tele.serialVersionUID:
        telePckt.serialVersionUID = tele.serialVersionUID
    else:
        telePckt.serialVersionUID = str(boardId)
    
    telePckt.rostime_secs = now.secs
    telePckt.rostime_nsecs = now.nsecs
    telePckt.flightTimeSecs = int(now.secs - initial_timestamp)
    
    telePckt.batteryThreashold = 10
    #print('int(batteryPercentage):', int(batteryPercentage))
    telePckt.batteryPercentage = int(batteryPercentage)
    if false_battery_metric > 0:
        telePckt.batteryPercentage = false_battery_metric
    
    telePckt.gpsSignal = tele.gpsSignal
    telePckt.satelliteNumber = tele.satelliteNumber

    if false_gps_metric:
        telePckt.satelliteNumber = false_gps_metric
    
    telePckt.altitude = tele.altitude
    telePckt.longitude = tele.longitude
    telePckt.latitude = tele.latitude
    telePckt.heading = tele.heading
    
    telePckt.velocity = tele.velocity
    # if telePckt.velocity < 0.05:
    #     telePckt.velocity = 0.0
    # if telePckt.velocity > 20.0:
    #     telePckt.velocity = 0.05
    
    telePckt.homeLatitude = home_latitude
    telePckt.homeLongitude = home_longitude
    
    #telePckt.droneState = calcDroneState(telePckt.altitude)
    telePckt.droneState = droneState
    telePckt.gimbalAngle = g_pitch
	
    if publisher:
        publisher.publish(telePckt)
        seq += 1


def publishPositioning(tele):
    global positioningPub

    positioningPacket = Positioning()
    positioningPacket.serialVersionUID = dronename
    positioningPacket.heading = tele.heading
    positioningPacket.velocity = tele.velocity
    
    positioningPacket.altitude = tele.altitude
    positioningPacket.latitude = tele.latitude
    positioningPacket.longitude = tele.longitude
    
    positioningPacket.d_roll = d_roll
    positioningPacket.d_pitch = d_pitch
    positioningPacket.d_yaw = d_yaw
    
    positioningPacket.g_roll = g_roll
    positioningPacket.g_pitch = g_pitch
    positioningPacket.g_yaw = g_yaw

    if positioningPub:
        positioningPub.publish(positioningPacket)
    

def telemetry_worker(frequency, default=False):
    global gpsTelemetry, dronename
    publish_rate = rospy.Rate(frequency)
    
    topic_name = dronename+'/Telemetry/' + str(frequency) + 'hz'
    if default:
        topic_name = dronename+'/Telemetry'

    publisher_worker = rospy.Publisher(topic_name, Telemetry, queue_size=1)


    # Publish empty telemetry to initialize drone position and avoid errors with the rest of the services
    if default:
        publisher_worker.publish(Telemetry())
    
    print('telemetry_worker starting up:', frequency, 'hz')

    while not rospy.is_shutdown():
        tele_packet, tele_source = psp.get_valid_telemetry()
        if tele_packet:
            
            if default:
                if not homePositionSet:
                    set_home_position(tele_packet)

            publishTelemetry(tele_packet, publisher_worker)


        # if gpsTelemetry:
        #     #print('telemetry_worker publishing')
        #     publishTelemetry(gpsTelemetry, publisher_worker)

        publish_rate.sleep()


def init():
    global homePositionPub
    rospy.init_node(dronename + '_telemetry_translator')
    
    initial_timestamp = rospy.Time.now().secs
    
    
    translatorPub = rospy.Publisher(dronename+'/Telemetry', Telemetry, queue_size=1)
    positioningPub = rospy.Publisher(dronename+'/Positioning', Positioning, queue_size=1)
    homePositionPub = rospy.Publisher(dronename+ '/Telemetry/HomePosition', Telemetry, queue_size=1, latch=True)
    
    rospy.Subscriber(dronename+'/attitude',QuaternionStamped, DroneOrientationCB)
    rospy.Subscriber(dronename+'/gimbal_angle',Vector3Stamped, GimbalOrientationCB)
    rospy.Subscriber(dronename+'/DroneState', String, DroneStateCB)    
    #rospy.Subscriber(dji_name+'/PSP/Telemetry', Telemetry, telemetryCB)

    # crps Telemetry
    #rospy.Subscriber('/crps/Telemetry', Telemetry, fusedTelemetryCB)

    rospy.Subscriber(dronename+'/battery_state', BatteryState, batteryCB)
    rospy.Subscriber(dronename+'/simulateBatteryFailure', Int8, battery_failure_CB)

    rospy.Subscriber(dronename+'/simulateGPSFailure', Int8, gps_failure_CB)

    worker50hz = threading.Thread(target=telemetry_worker, args=(50, True))
    worker50hz.start()
    
    worker2hz = threading.Thread(target=telemetry_worker, args=(2, ))
    worker2hz.start()

    psp.init()
    print('\nTranslator READY', 'FROM:', dronename+'/telemetry2', 'TO:', dronename+'/Telemetry', '\n')


def listener(dji_name = "matrice300"):
    global boardId, translator, home_latitude, home_longitude
    global dronename
    global positioningPub
    global initial_timestamp

    dronename = dji_name.replace('/', '')
    boardId = dji_name.split('_', 1)[1]
    print('TelemetryTranslator boardId:', boardId)

    init()

    rospy.spin()


# if __name__ == '__main__':
# 	try:
# 		rospy.init_node('jetson300translator', anonymous=False)
# 		private_param = rospy.get_param('~dji_name', "matrice300")
# 		listener(dji_name = private_param)
# 	except rospy.ROSInterruptException:
# 		pass
if __name__=='__main__':
    print('Initializing Telemetry Translator...')
    f = open("/etc/machine-id", "r")
    boardId = f.read().strip()[0:4]
    
    dronename = '/matrice300' + '_' + boardId
    listener(dronename)
