#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import signal
import subprocess
import datetime
import socket

import rospy

from dji_sdk.msg import telemetry2, Packet, Packet2, Gimbal
from dji_sdk.msg import EscData, ESCStatusIndividual
from dji_sdk.msg import ComponentList, ComponentState
from dji_sdk.srv import GetSingleBatteryDynamicInfo

from kios.msg import Telemetry, TerminalHardware, DroneHardware
from std_msgs.msg import String, Float64MultiArray, Bool
from sensor_msgs.msg import Imu, NavSatFix, BatteryState
from geometry_msgs.msg import Vector3Stamped, QuaternionStamped


ros_version = subprocess.check_output('rosversion -d', shell=True).decode('utf-8').strip()
print('ros_version:', ros_version)
if ros_version == 'melodic':
    #print('multimaster_msgs_fkie')
    from multimaster_msgs_fkie.msg import LinkStatesStamped
elif ros_version == 'noetic':
    #print('fkie_multimaster_msgs')
    from fkie_multimaster_msgs.msg import LinkStatesStamped


file_path = os.path.realpath(__file__)
file_path = file_path.split('src', 1)[0] + 'launch/'

print(file_path)
sys.path.insert(1, file_path)

import getActiveIP

def GetBoardID():
    f = open("/var/lib/dbus/machine-id", "r")
    boardID = f.read().strip()[0:4]
    #print(boardID)
    return boardID


boardId = GetBoardID()
dronename = '/matrice300' + '_' + boardId

print(dronename)

droneState = "On Ground"
droneIdAck = False
droneIDpub = None


hostname = socket.getfqdn()
localIp = None
platformIp = None

def signal_handler(sig, frame): 
    global droneIDpub
    hello_msg = "{\"Timestamp\":\""+str(rospy.get_rostime())+"\",\"DroneName\":\""+dronename.replace('/','')+"\",\"Connected\":\"False\",\"Model\":\"MATRICE_300_RTK\",\"cameraVideoStreamSource\":\"unknown\",\"cameraName\":\"Zenmuse_H20T\"}"
    
    if droneIDpub:
        droneIDpub.publish(hello_msg)

    print('Connected: False sent to platform.')
    sys.exit(0)

    
def DroneIdAckCB(ack):
    global droneIdAck
    
    print('Ack received', ack)
    
    if ack.data == '1':
        droneIdAck = True
                  
            
def checkPortStatus(ip, port):
    p = subprocess.Popen(("nc", "-zvw5", ip, port), stderr = subprocess.PIPE)
    status = p.communicate()[1].decode("utf-8")
    
    #print(str(status))
    # print (rc)
    # print (status)

    if 'succeeded' in str(status):
        return True
    else:
        return False


def getLocalIp():
    ip = getActiveIP.getActiveIp()
    
    if not ip:
        print('Invalid ip value')
        
    return ip
    

def getPlatformIp():
    global localIp
    print('Finding platform Ip...')
    platformIp = None

    print_c = 0
    print_c_max = 2

    while not platformIp and not rospy.is_shutdown():
        linkStats = rospy.wait_for_message('/master_discovery/linkstats', LinkStatesStamped)
        rosdevices = linkStats.links
        
        for device in rosdevices:          
            if rospy.is_shutdown():
                break
            
            if device.destination != localIp:
                #print('testing rospy device:', device.destination, 'local ip:', localIp)
                if checkPortStatus(device.destination, '8000') and checkPortStatus(device.destination, '1935'):
                    platformIp = device.destination
                    print("Platform Ip:", device)
                    return platformIp

            
        if not rospy.is_shutdown():
            rospy.sleep(0.1)
            if print_c < print_c_max:
                print('connectionHandler - Polling platform...')
                print_c = print_c + 1
                if print_c >= print_c_max:
                    print('connectionHandler - Polling platform silently...')


def handshakePlatform():
    global droneIdPub, droneIdAckSub, droneIdAck
    
    droneIdAckSub = rospy.Subscriber(dronename+'/handshake',String, DroneIdAckCB)
      
      
    hello_msg = "{\"Timestamp\":\""+str(rospy.get_rostime())+"\",\"DroneName\":\""+dronename.replace('/','')+"\",\"Connected\":\"True\",\"Model\":\"MATRICE_300_RTK\",\"cameraVideoStreamSource\":\"unknown\",\"cameraName\":\"Zenmuse_H20T\"}"
    timeout = 1000000
    print(hello_msg)
    for i in range(0, timeout):
        if rospy.is_shutdown():
            print('Rospy shut down. Handshake process terminated')
            return False
        
        droneIdPub.publish(hello_msg)
        print('Attempt:\t' + str(i))
        rospy.sleep(5)
        
        if droneIdAck:
            print('Platform register Successful')
            droneIdAck = False
            return True
        
        if rospy.is_shutdown():
            break
 
    if not droneIdAck:
        print('Could not register on platform.\n') 
        return False


def pollPlatformConnection(platformIp):
    global platformIpPub, platform_connected_Pub
    
    response = os.system("ping -c 1 " + platformIp  + " > /dev/null 2>&1")
    
    pingPort = 0

    #and then check the response...
    if response == 0:
      #print(platformIp, 'is up!')
      pingPort = 1
    else:
      print(platformIp, 'is down!')
      
    #buildmapPort = checkPortStatus(platformIp, '8000')
    #streamPort = checkPortStatus(platformIp, '1935')
    
    connectionGood = pingPort
    #connectionGood = pingPort * buildmapPort * streamPort
    #print('connectionGood:', connectionGood, datetime.datetime.now().strftime("%Y%m%d-%H:%M:%S.%s"))
    
    if connectionGood:
        platform_connected_Pub.publish(True)
        platformIpPub.publish(platformIp)       
    else:
        platform_connected_Pub.publish(False)
        platformIpPub.publish("None")
        
    #print('platform ip published')
    
    return connectionGood


def platform_disconnect():
    global dronename
    global droneIdPub
    goodbye_msg = "{\"Timestamp\":\""+str(rospy.get_rostime())+"\",\"DroneName\":\""+dronename.replace('/','')+"\",\"Connected\":\"False\",\"Model\":\"MATRICE_300_RTK\",\"cameraVideoStreamSource\":\"unknown\",\"cameraName\":\"Zenmuse_H20T\"}"
    
    print('\nPublishing goodbye_msg:', goodbye_msg)
    droneIdPub.publish(goodbye_msg)
    droneIdPub.publish(goodbye_msg)
    droneIdPub.publish(goodbye_msg)
    print('Goodbye Message Published.\n')


def init():
    global dronename
    global platformIpPub, platform_connected_Pub
    global droneIdPub

    platformIpPub = rospy.Publisher(dronename+'/platform/Platform_IP', String, queue_size=1, latch=True)

    platform_connected_Pub = rospy.Publisher(dronename+'/platform/Platform_Connected', Bool, queue_size=1, latch=True)
    platform_connected_Pub.publish(False)

    droneIdPub = rospy.Publisher('/droneIds', String, queue_size=1)
    platform_disconnect()

    
def listener(dji_name = 'matrice300'):
    global dronename, localIp, platformIp, platformIpPub, componentListPub
    
    print('Initializing connection handler...')
    dronename = dji_name
    
    rospy.init_node(dronename + '_platform_connection_handler')

    rate = rospy.Rate(0.05)
    rate.sleep()

    init()
    
    print('Connection Handler - Listening for available components...')
    
    
    
    localIp = getLocalIp()
    print('Connection Handler - localIp:\t' + localIp)
    
    while not rospy.is_shutdown():   
        platformIp = getPlatformIp()

        if handshakePlatform():
            signal.signal(signal.SIGINT, signal_handler)
        else:
            print('Could not shake hands with platform...\nExiting...\n')
            exit(0)
        
        platformConnected = pollPlatformConnection(platformIp)        
        while platformConnected and not rospy.is_shutdown(): 
            rate.sleep()
            platformConnected = pollPlatformConnection(platformIp)
            
        platformIpPub.publish("None")
          
        print('Lost platform connectivity...')
        print('Retrying...')


if __name__=='__main__':
    listener('matrice300_' + GetBoardID())
