#!/usr/bin/env python
import logging
import os
import stat
import sys
import threading
import time
from optparse import OptionParser

import atexit

import numpy as np
import rospy
import serial

from dji_sdk.msg import ComponentState
from kios.msg import trisonica_msg


def pollAvailability(frequency = 0.1):
    usbname = "FTDI_FT230X_Basic_UART_D307A366"
    
    rate = rospy.Rate(frequency)

    while not rospy.is_shutdown():
        for name in os.listdir("/dev/serial/by-id/"):
             if usbname in name:
                 print('Found weather station at: ' + '/dev/serial/by-id/' + name)
                 return '/dev/serial/by-id/' + name
             
        rate.sleep()
        print('Polling for weather station port...')


freq = 1
devicePort = None
def establishConnection(port="/dev/ttyUSB0", topic='/trisonica', baud=115200, rate=1):
    global devicePort, componentStatePub
    
    rate = rate # rate to check/publish new data, Hz
    devicePort = port
    baud = baud
    
    print('Connecting to Weather Station...')    
    componentState = ComponentState()
    componentState.componentName = 'Weather Station'
    componentState.componentPort = devicePort
    componentState.componentActive = True
    
    #print('componentState:', componentState)
    
    componentStatePub.publish(componentState)
    
    try:     
        connection = serial.Serial(devicePort, baud, timeout=0.01)
        connection.flush() 
        
        atexit.register(disconnectDevice)
        
        publishWeatherStationData(connection)
    except serial.serialutil.SerialException as e:
        print('Weather Station error:', e)
        # print(e)
        #logger.error(e)
        #logger.info("No weather station plugged on this PC. Waiting.")
        # print("No weather station plugged on this PC. Waiting...")
        #self.keepRetrying()

        
def disconnectDevice():
    global devicePort, componentStatePub
    
    componentState = ComponentState()
    componentState.componentName = 'Weather Station'
    componentState.componentPort = devicePort
    componentState.componentActive = False
    
    componentStatePub.publish(componentState)
        

def publishWeatherStationData(connection):
    global freq
    msg = trisonica_msg()

    rate = rospy.Rate(freq) # Hz, trisonica set to 40 Hz
    
    publisher = rospy.Publisher(dronename+'/WeatherStation', trisonica_msg, queue_size=10)
    
    print('Weather Station Connected')

    while not rospy.is_shutdown():
        """msg.header.stamp.secs = rospy.Time.now().secs
            msg.header.stamp.nsecs = rospy.Time.now().nsecs
            msg.speed=2
            msg.speed2d=3
            msg.direction=4
            msg.northsouth=5
            msg.westeast=6
            msg.updown= 7
            msg.temperature=8
            msg.pressure=9
            msg.humidity=10
            msg.pitch =11
            msg.roll=12
            msg.heading=13
            msg.levelx=14
            msg.levely=15
            msg.levelz=16

        self.publisher.publish(msg) """

        try:
            data = connection.readline()
        except serial.serialutil.SerialException as e:
            #logger.warning("Seems like weather station was disconnected!")
            #logger.info("Waiting for weather station to reconnect.")
            # print("Seems like weather station was disconnected! ")
            # print("Waiting for weather station to reconnect...")
            connection.close()
            return
            #data = keepRetrying()


        if data is not None and len(data) > 10:
            if 1: #data[0] == 'S':
                msg.header.stamp.secs = rospy.Time.now().secs
                msg.header.stamp.nsecs = rospy.Time.now().nsecs
                try:
                    msg.speed       = float( data.decode().split('S ')[1].lstrip().split(' ')[0] )
                except:
                    #msg.speed = np.nan 
                    pass

                try:
                    msg.speed2d       = float( data.decode().split('S2 ')[1].lstrip().split(' ')[0] )
                except:
                    #msg.speed2d = np.nan  
                    pass

                try:
                    msg.direction   = float( data.decode().split('D ')[1].lstrip().split(' ')[0])
                except:
                    #msg.direction = np.nan   
                    pass

                try:
                    msg.northsouth  = float( data.decode().split('U ')[1].lstrip().split(' ')[0] )
                except:
                    #msg.northsouth = np.nan  
                    pass

                try:
                    msg.westeast    = float( data.decode().split('V ')[1].lstrip().split(' ')[0] )
                except:
                    #msg.westeast = np.nan  
                    pass

                try:
                    msg.updown      = float( data.decode().split('W ')[1].lstrip().split(' ')[0] )
                except:
                    #msg.updown = np.nan  
                    pass

                try:
                    msg.temperature = float( data.decode().split('T ')[1].lstrip().split(' ')[0] )
                except:
                    #msg.temperature = np.nan  
                    pass

                try:
                    msg.pressure = float( data.decode().split('P ')[1].lstrip().split(' ')[0] )
                except:
                    #msg.pressure = np.nan  
                    pass

                try:
                    msg.humidity = float( data.decode().split('H ')[1].lstrip().split(' ')[0] )
                except:
                    #msg.humidity = np.nan   
                    pass

                try:
                    msg.pitch = float( data.decode().split('P ')[2].lstrip().split(' ')[0] )
                except:
                    try:
                        msg.pitch = float( data.decode().split('PI ')[1].lstrip().split(' ')[0] )
                    except:
                        #msg.pitch = np.nan  
                        pass

                try:
                    msg.roll = float( data.decode().split('RO ')[1].lstrip().split(' ')[0] )
                except:
                    #msg.roll = np.nan   
                    pass

                try:
                    msg.heading = float( data.decode().split('MD ')[1].lstrip().split(' ')[0] )
                except:
                    #msg.heading = np.nan   
                    pass

                try:
                    msg.levelx = float( data.decode().split('AX ')[1].lstrip().split(' ')[0] )
                except:
                    #msg.levelx = np.nan   
                    pass

                try:
                    msg.levely = float( data.decode().split('AY ')[1].lstrip().split(' ')[0] )
                except:
                    #msg.levely = np.nan   
                    pass

                try:
                    msg.levelz = float( data.decode().split('AZ ')[1].lstrip().split(' ')[0] )
                except:
                    #msg.levelz = np.nan   
                    pass 


                # print("MSG: ", msg)
                publisher.publish(msg)
                # print("==========================================================")
        rate.sleep()
    connection.close()
 

def listener(dji_name = "/matrice300"):
    global componentStatePub

    
    dronename = dji_name
    
    print('Setting up node: ' + dronename + '_weather_station')
    rospy.init_node(dronename + '_weather_station')
    
    componentStatePub = rospy.Publisher(dronename+'/ComponentState', ComponentState, queue_size=1, latch=True)
    
#    componentState = ComponentState()
#    componentState.componentName = 'Weather Station'
#    componentState.componentPort = devicePort
#    componentState.componentActive = True
#    
#    componentStatePub.publish(componentState)
#    componentStatePub.publish(componentState)
        
    #weatherStationPort = pollAvailability()        
    #establishConnection(port = weatherStationPort)
    establishConnection()
    while not rospy.is_shutdown():
        rospy.spin()


if __name__ == '__main__':
    global dronename
    
    print('Initializing Weather Station...')
    f = open("/var/lib/dbus/machine-id", "r")
    boardId = f.read().strip()[0:4]
    
    dronename = 'matrice300' + '_' + boardId
    
    listener(dronename)
