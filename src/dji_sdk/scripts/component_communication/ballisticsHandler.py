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

from sensor_msgs.msg import Joy


devicePort = None
def findDevicePort():
    global devicePort
    #DEV_ID=$(lsusb | grep 'QinHeng Electronics HL-340 USB-Serial adapter' | awk '{ print $6 }' | awk -F ':' '{ print $1 }')
	#TTY_PORT=$(ls -l /dev/serial/by-id | grep $DEV_ID | awk -F '/' '{ print $3}')
	#rosrun rosserial_python serial_node.py /dev/$TTY_PORT &
    
    deviceID = subprocess.check_output(('lsusb | grep \'QinHeng Electronics HL-340 USB-Serial adapter\' | awk \'{ print $6 }\' | awk -F \':\' \'{ print $1 }\''), shell=True).strip()
    print('device id:', deviceID)
    
    ttyPort = subprocess.check_output(('ls -l /dev/serial/by-id | grep ' + deviceID +' | awk -F \'/\' \'{ print $3}\''), shell=True).strip()
    devicePort = '/dev/' + ttyPort
    print('Ballistics devicePort:', devicePort)


deviceConnected = False
def connectDevice():
    global deviceConnected, devicePort
    
    baud = 57600
    
    #p = subprocess.Popen('rosrun rosserial_python serial_node.py ' + dev, stdout=subprocess.PIPE)
    command = 'rosrun rosserial_python serial_node.py ' + '_port:=' + '/dev/ttyUSB0' + ' _baud:=' + str(baud) + ' &'
    print(command)
    subprocess.call('rosrun rosserial_python serial_node.py ' + '_port:=' + '/dev/ttyUSB0' + ' _baud:=' + str(baud) + ' &', shell=True)
    deviceConnected = True
    print('Ballistics device connected.\n\tSerial Port:\t' + devicePort + '\n\tBaud Rate:\t' + str(baud))
        
    componentState = ComponentState()
    componentState.componentName = 'Ballistic'
    componentState.componentPort = devicePort
    componentState.componentActive = True
    
    componentStatePub.publish(componentState)
   
    
def disconnectDevice():
    componentState = ComponentState()
    componentState.componentName = 'Ballistic'
    componentState.componentPort = devicePort
    componentState.componentActive = False
    
    componentStatePub.publish(componentState)


def ballisticsCB(cmd):
    firingMode = cmd.data
    
    print('Received Ballistics Command:', firingMode)
    
    if firingMode > 1:
        fireMultiple()
    else:
        fireSingle()

        
triggerState = 0
def fireBallisticsCB(joyInput):
    global triggerState
    
    if joyInput.buttons[2] > 0 and not triggerState:
        triggerState = 1
        print('Pressed FIRE SINGLE button')
        fireSingle()        
    elif joyInput.buttons[3] > 0 and not triggerState:
        triggerState = 1
        print('Pressed FIRE MULTIPLE button')
        fireMultiple()
    elif joyInput.buttons[2] < 1 and joyInput.buttons[3] < 1:
        triggerState = 0


def fireSingle():
    rospy.Publisher('/matrice300/ballistics', String, queue_size=1).publish("f")
    
def fireMultiple():
    rospy.Publisher('/matrice300/ballistics', String, queue_size=1).publish("m")


def listener(dji_name = "/matrice300"):
    global dronename, waterSamplerPub, componentStatePub
    global rate
    
    dronename = dji_name
    nodename = dronename + '_balistics_handler'
    
    rospy.init_node(nodename)
    rate = rospy.Rate(0.5)
    
    findDevicePort()
    connectDevice()
    
    rospy.Subscriber(dronename + '/Ballistics', Int32, ballisticsCB)
    rospy.Subscriber(dronename + '/joy', Joy, fireBallisticsCB)
    
    componentStatePub = rospy.Publisher(dronename+'/ComponentState', ComponentState, queue_size=1, latch=True)
    atexit.register(disconnectDevice)
    
    print('Ballistics Handler Ready')     
    rospy.spin()


if __name__ == '__main__':
    global dronename
    
    print('Initializing Ballistics Handler...')
    f = open("/var/lib/dbus/machine-id", "r")
    boardId = f.read().strip()[0:4]
       
    dronename = 'matrice300' + '_' + boardId
    
    listener(dronename)
