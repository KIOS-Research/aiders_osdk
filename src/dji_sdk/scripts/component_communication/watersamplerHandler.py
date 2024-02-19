#!/usr/bin/env python
import logging
import os
import stat
import sys
import threading
import time
import subprocess
from optparse import OptionParser

import numpy as np
import rospy
import serial

import atexit

from std_msgs.msg import String, Int32, Float32
from dji_sdk.msg import ComponentState
from kios.msg import Telemetry



devicePort = None
def findDevicePort():
    global devicePort
    #DEV_ID=$(lsusb | grep 'QinHeng Electronics HL-340 USB-Serial adapter' | awk '{ print $6 }' | awk -F ':' '{ print $1 }')
	#TTY_PORT=$(ls -l /dev/serial/by-id | grep $DEV_ID | awk -F '/' '{ print $3}')
	#rosrun rosserial_python serial_node.py /dev/$TTY_PORT &
    
    deviceID = subprocess.check_output(("lsusb | grep 'QinHeng Electronics HL-340 USB-Serial adapter' | awk '{ print $6 }' | awk -F ':' '{ print $1 }'"), shell=True).strip()
    
    if '\n' in deviceID:
        deviceID = deviceID.split()[0]
    
    
    print('device id:', deviceID)
    
    ttyPort = subprocess.check_output(("ls -l /dev/serial/by-id | grep " + deviceID + " | awk -F '/' '{ print $3}'"), shell=True).strip()
    print('ttyPort:', ttyPort)
    
    devicePort = '/dev/' + ttyPort


deviceConnected = False
def connectDevice():
    global deviceConnected, devicePort
    
    #p = subprocess.Popen('rosrun rosserial_python serial_node.py ' + dev, stdout=subprocess.PIPE)
    subprocess.call('rosrun rosserial_python serial_node.py ' + devicePort + ' &', shell=True)
    deviceConnected = True
    print('Water Sampler device connected.\n\tSerial Port:\t' + devicePort)
    
    componentState = ComponentState()
    componentState.componentName = 'Water Sampler'
    componentState.componentPort = devicePort
    componentState.componentActive = True
    
    componentStatePub.publish(componentState)
    
    
def disconnectDevice():    
    componentState = ComponentState()
    componentState.componentName = 'Water Sampler'
    componentState.componentPort = devicePort
    componentState.componentActive = False
    
    componentStatePub.publish(componentState)


watersamplerActive = False    
def watersamplerSolenoidCB(cmd):
    global waterSamplerPub
    setActive = cmd.data
    
    print('cmd:', cmd)
    
    if setActive:
        openWaterSampler()
    else:
        closeWaterSampler()
        
        
def watersamplerSensorCB(reading):
    waterCapacitanceLimit = 400
    
    if reading.data < waterCapacitanceLimit:
        dronePositioning = rospy.wait_for_message(dronename + '/Telemetry', Telemetry)
        minimumAltitudePacket = Float32()
        minimumAltitudePacket.data = dronePositioning.altitude
        
        rospy.Publisher(dronename + '/setMinimumAltitude', Float32, queue_size=1).publish(minimumAltitudePacket)
    else:
        minimumAltitudePacket = Float32()
        minimumAltitudePacket.data = 0.0
        
        rospy.Publisher(dronename + '/setMinimumAltitude', Float32, queue_size=1).publish(minimumAltitudePacket)
        
    #print(reading)
    #print(reading.data)
        
        
shutoffThread = None
def openWaterSampler():
    global shutoffThread
    global watersamplerActive
    wsCmd = String()
    wsCmd.data = 'o'
    print(wsCmd)
    waterSamplerPub.publish(wsCmd)
    
    notificationMsg = String()
    notificationMsg.data = 'Water Sampler - Opened Solenoid'
    
    notificationPub.publish(notificationMsg)
    
    
    watersamplerActive = True
    
    if shutoffThread is None:
        print('Water sampler timer initialized')
        shutoffThread = threading.Thread(target=watersamplerShutoffTimer)
        shutoffThread.start()
    
    
def closeWaterSampler():
    global watersamplerActive
    
    wsCmd = String()
    wsCmd.data = 'c'
    print(wsCmd)
    waterSamplerPub.publish(wsCmd)
    
    notificationMsg = String()
    notificationMsg.data = 'Water Sampler - Closed Solenoid'
    
    notificationPub.publish(notificationMsg)
    
    watersamplerActive = False
    
    
    

def watersamplerShutoffTimer():
    global shutoffThread
    global solenoid_timer_Pub
    
    startTime = time.time()
    timeout = 20    # seconds
    
    timediff = time.time() - startTime
    while watersamplerActive and timediff < timeout:
        timediff = time.time() - startTime
        solenoid_timer_Pub.publish(timediff)
        
        rate.sleep()

    timediff = float(time.time() - startTime)
    solenoid_timer_Pub.publish(timediff)
        
    print('Water Sampler timer: Closing sampler')
    closeWaterSampler()
    shutoffThread = None
        

    

def listener(dji_name = "/matrice300"):
    global dronename, waterSamplerPub, notificationPub, componentStatePub
    global solenoid_timer_Pub
    global rate
    
    dronename = dji_name
    nodename = dronename + '_water_sampler_handler'
    
    rospy.init_node(nodename)
    rate = rospy.Rate(1.0)
    
    rospy.Subscriber(dronename + '/WaterSampler', Int32, watersamplerSolenoidCB)
    rospy.Subscriber('/matrice300/wsmSensor', Int32, watersamplerSensorCB)
    waterSamplerPub = rospy.Publisher('/matrice300/wsmSolenoid', String, queue_size=1)
    solenoid_timer_Pub = rospy.Publisher(dronename+'/watersampler/solenoid_time_till_close', Float32, queue_size=1, latch=1)
    solenoid_timer_Pub.publish(0.0)
    
    notificationPub = rospy.Publisher(dronename + "/Error", String, queue_size=10)
    
    componentStatePub = rospy.Publisher(dronename+'/ComponentState', ComponentState, queue_size=1, latch=True)

    #waterSamplerPub = rospy.Publisher('/matrice300/command', String, queue_size=1)
    #waterSamplerPub = rospy.Publisher('/matrice300/command', String, queue_size=1)
    connectDevice()
    atexit.register(disconnectDevice)

    
    print('Water Sampler Handler Ready')   
    
    
    while not rospy.is_shutdown():
        rospy.spin()
#        wsCmd = String()
#        wsCmd.data = 'o'
#        print(wsCmd)
#        waterSamplerPub.publish(wsCmd)
#        rate.sleep()
#        
#        wsCmd = String()
#        wsCmd.data = 'c'
#        print(wsCmd)
#        waterSamplerPub.publish(wsCmd)
#        rate.sleep()
         

if __name__ == '__main__':
    global dronename
    
    print('Initializing Water Sampler Handler...')
    f = open("/var/lib/dbus/machine-id", "r")
    boardId = f.read().strip()[0:4]
    
    findDevicePort()
    
    dronename = 'matrice300' + '_' + boardId
    
    listener(dronename)
