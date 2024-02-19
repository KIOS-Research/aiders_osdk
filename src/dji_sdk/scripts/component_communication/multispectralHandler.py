#!/usr/bin/env python
import os
import sys
import subprocess
import numpy as np
import time
import rospy
import threading
import requests
import io
import shutil
#import urllib
#from six.moves import urllib
import urllib2
import glob


from datetime import datetime

from std_msgs.msg import String
from kios.msg import Telemetry, BuildMap
from geometry_msgs.msg import Vector3Stamped

multispectralIp = "192.168.10.254"
buildMapImgPort = '8000'
multispectralActive = False

outputDirectory = 'input_images/'


latitude = 0.0
longitude = 0.0
altitude = 0.0
velocity = 0.0
def telemetryCB(telemetry):
    global altitude, latitude, longitude, bearing, velocity
    altitude = float(telemetry.altitude)
    latitude = float(telemetry.latitude)
    longitude = float(telemetry.longitude)


vel_n = 0.0
vel_e = 0.0
vel_d = 0.0
def velocityCB(velocity):
    global vel_n, vel_e, vel_d
    vel_e = velocity.vector.x
    vel_n = velocity.vector.y
    vel_d = velocity.vector.z


multispectralGpsThread = None
multispectralImageThread = None
def multispectralStateCB(state):
    global multispectralActive, overlap
    stateActive = state.buildmap
    overlap = state.overlap / 100.0

    print('multispectralActive: ', stateActive)
    print('overlap: ', overlap)

    if stateActive != multispectralActive:
        multispectralActive = stateActive
        
        if multispectralActive:
            multispectralGpsThread = threading.Thread(target=pushMultispectralMetadata)
            multispectralGpsThread.start()
            
            multispectralImageThread = threading.Thread(target=captureMultispectralStream)
            multispectralImageThread.start()
        else:          
            #multispectralGpsThread.join()
            #multispectralImageThread.join()
            pass
    


def pushMultispectralMetadata():
    print('Updating GPS data of multispectral camera...')
   	
    while multispectralActive:
        gps_params = {'latitude' : latitude, 
                  'longitude' : longitude, 
                  'altitude' : altitude,
                  'p_acc' : 1,
                  'vel_n' : vel_n,
                  'vel_e' : vel_e,
                  'vel_d' : vel_d,
                  'v_acc' : 1,
                  'fix3d' : True }
        
        gps_data = requests.post("http://192.168.10.254/gps", json=gps_params)
        print('gps_data:', gps_data)
        time.sleep(1)
        
    print('Updating GPS data of multispectral camera finished.')
                

def captureMultispectralStream():   
    capture_params = {'store_capture': True, 'block': True}
    print(capture_params)
    
    if not os.path.isdir(outputDirectory):
        os.makedirs(outputDirectory)
    
    bandThreads = [None] * 6  
    
    while multispectralActive:
        capture_data = requests.post('http://' + multispectralIp + '/capture', json=capture_params)   
        print('capture_data:',capture_data)
        print('capture_data:',capture_data.json())
        
        timestamp = int(time.time() * 10**10)
        for bandNumber in range(1, 7):
            bandThreads[bandNumber-1] = threading.Thread(target=captureMultispectralBand, args=(bandNumber, capture_data.json()['raw_storage_path'][str(bandNumber)], timestamp))
            bandThreads[bandNumber-1].start()
            
        for bandNumber in range(1, 7):
            bandThreads[bandNumber-1].join()    
    
        URL = "http://192.168.10.254/" + capture_data.json()['raw_storage_path']['2']
        time.sleep(1)
        
instanceMutex = threading.Lock()
def captureMultispectralBand(bandNumber, storagePath, timestamp):
    URL = 'http://' + multispectralIp + '/' + storagePath
    
    multispectralInstanceDir = outputDirectory + "multispectral_" + str(timestamp) + '/'
    
    instanceMutex.acquire()
    if not os.path.isdir(multispectralInstanceDir):
        os.makedirs(multispectralInstanceDir)
    instanceMutex.release()
    
    photoName = "multispectral_" + str(timestamp) + "_" + str(bandNumber) + ".tif"
    photoPath = multispectralInstanceDir + photoName
            
    print('Designated Image Path:', photoPath)
    multispectralUrl = urllib2.urlopen(URL)
    byteImg = io.BytesIO(multispectralUrl.read())
    byteImg.seek(0)

    with open(photoPath, 'wb') as f:
        shutil.copyfileobj(byteImg, f)

    print("Band number " + str(bandNumber) + " TIF FILE SAVED TO " + photoPath)


def postImage(imageDirectory):
    global altitude, latitude, longitude, bearing, imgDirectory, platform_ip, buildMapImgUrl
    global d_roll, d_pitch, d_yaw
    global g_roll, g_pitch, g_yaw

    print('Image directory: ' + imageDirectory)
    print('Posting Image: ' + imgName)
    payload = {'image_name':imgName,
           'drone_name':dronename.replace('/', ''),
           'bearing': bearing,
           'alt':altitude,
           'lat':latitude,
           'lon':longitude,
           'd_roll':d_roll,
           'd_pitch':d_pitch,
           'd_yaw':d_yaw,
           'g_roll':g_roll,
           'g_pitch':g_pitch,
           'g_yaw':g_yaw}


    myfile=imgDirectory + imgName

    print('Image Full Directory:', imageDirectory)
    files = {}

    

    #files = {'image_file': open(myfile, 'rb')}
    
    c = 0
    for band in glob.glob(imageDirectory):
        files['imagefile' + str(c)].append(open(band, 'rb'))
        
    print('Len of files:', len(files))
        
    if platform_ip:
        if not buildMapImgUrl:
            buildMapImgUrl = 'http://' + platform_ip + ':' + buildMapImgPort + '/postBuildMapImg/'       
            #print ('buildMapImgUrl:', buildMapImgUrl)      
        elif platform_ip not in buildMapImgUrl:
            buildMapImgUrl = 'http://' + platform_ip + ':' + buildMapImgPort + '/postBuildMapImg/'       
            #print ('buildMapImgUrl:', buildMapImgUrl)
    
        try:
            requests.post(buildMapImgUrl, data=payload, files=files)
        except Exception as e:
            print('Post Image to URL', e)
    
    
def listener(dji_name = "/matrice300"):
    global dronename
    dronename = dji_name
    boardId = dji_name.split('_', 1)[1]
    
    nodename = dronename.replace('/', '') + '_multispectral' 
    print('MultispectralHandler', nodename)
    rospy.init_node(nodename)
    
    # Platform Ip
    rospy.Subscriber(dronename+"/MultispectralBuildMapRequest", BuildMap, multispectralStateCB)
    rospy.Subscriber(dronename+'/Telemetry', Telemetry, telemetryCB)
    rospy.Subscriber(dronename+'/velocity', Vector3Stamped, velocityCB)
    
    while not rospy.is_shutdown():
        rospy.spin()


if __name__=='__main__':
    print('Initializing Multispectral Handler...')
    f = open("/var/lib/dbus/machine-id", "r")
    boardId = f.read().strip()[0:4]
    
    dronename = '/matrice300' + '_' + boardId
    listener(dronename)  