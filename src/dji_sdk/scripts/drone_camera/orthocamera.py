#!/usr/bin/env python
import os
import rospy
import time
import math
import threading
import requests
import numpy as np

#import tf.transformations
import execnet
def call_python_version(Version, Module, Function, ArgumentList):
    gw      = execnet.makegateway("popen//python=python%s" % Version)
    channel = gw.remote_exec("""
        from %s import %s as the_function
        channel.send(the_function(*channel.receive()))
    """ % (Module, Function))
    channel.send(ArgumentList)
    return channel.receive()



import copy
#import ffmpeg
from kios.msg import Telemetry, BuildMap

from dji_sdk.msg import Resolution
from dji_sdk.srv import SetupCameraStream, SetupCameraH264, CameraStartShootSinglePhoto, GimbalAction

from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Header, Time, String
from geometry_msgs.msg import Vector3Stamped, Vector3, QuaternionStamped

from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply

from threading import Thread, Lock

#from PIL import Image as pilImage
#import piexif
#from GPSPhoto import gpsphoto

import exiftool
import atexit

saveFrameMutex = Lock()

# OpenCV2 for saving an image
import cv2
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()     # Initialize Bridge

#import rtmpHandler

from geopy import Point
from geopy.distance import geodesic

rad2deg = 57.2958
deg2rad = 0.0174533


file_path = os.path.realpath(__file__)
file_path = file_path.split('src', 1)[0]
print(file_path)

imgDirectory = file_path + 'input_images/'

mainCamImageSub = None


platform_ip = None
buildMapImgPort = '8000'
buildMapImgUrl = None
platformIpSub = None
def platformIpCB(platformIp):
    global platform_ip, buildMapImgPort, buildMapImgUrl
    
    if platformIp.data == 'None':
        platform_ip = None      
        print('orthocamera - received None platform Ip')
    elif str(platformIp.data) == str(platform_ip): 
        #print('orthocamera - received identical platform Ip')
        pass
    else:
        print('orthocamera - received new platform Ip', 'old:', platform_ip, 'new:', platformIp.data)
        platform_ip = platformIp.data   


bearing = 0.0
altitude = 0.0
latitude = 0.0
longitude = 0.0
velocity = 0.0
def telemetryCB(telemetry):
    global altitude, latitude, longitude, bearing, velocity
    altitude = float(telemetry.altitude)
    latitude = float(telemetry.latitude)
    longitude = float(telemetry.longitude)
    bearing = float(telemetry.heading)

    velocity = float(telemetry.velocity)


g_roll = 0.0
g_pitch = 0.0
g_yaw = 0.0
def GimbalCB(gimbal):
    global g_roll, g_pitch, g_yaw
    g_roll = gimbal.vector.y
    g_pitch = gimbal.vector.x
    g_yaw = gimbal.vector.z
    #print (g_roll, g_pitch, g_yaw)


d_roll = 0.0
d_pitch = 0.0
d_yaw = 0.0
def DroneOrientationCB(msg):
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
    #print (d_roll, d_pitch, d_yaw)


cameraThread = None
saveFrame = False
orthophotographyActive = False
#/BuildMapRequest
def cameraStateCB(state):
    global cameraThread, orthophotographyActive, overlap
    cameraStateActive = state.buildmap
    overlap = state.overlap / 100.0

    print('cameraStateActive: ', cameraStateActive)
    print('overlap: ', overlap)

    if cameraStateActive and not orthophotographyActive:
        print('Initializing orthophotography thread')
        orthophotographyActive = True
        cameraThread = threading.Thread(target=orthophotography)
        cameraThread.start()

    if not cameraStateActive and orthophotographyActive:
        print('Terminating orthophotography thread')
        orthophotographyActive = False


avposCount = 0
avposStuck = 200
def orthophotography():
    global orthophotographyActive, mainCamImageSub
    global saveFrame, photoPosValid
    
    global avposCount, avposStuck

    print('=== Orthophotography ===')
    angleCameraOrthophotography()
    while abs(g_pitch - -90) > 3.0:
        time.sleep(1)
    
    rate = rospy.Rate(15)
    while orthophotographyActive and not rospy.is_shutdown():
        saveFrameMutex.acquire()
        if not saveFrame:
            if orthophotographyActive:     
                print('Orthophotography saveFrame = True', threading.current_thread())
                saveFrame = True
                photoPosValid = False
             
            saveFrameMutex.release()            
            rate.sleep()
            
            while not photoPosValid and not rospy.is_shutdown():
                
                avposCount = avposCount + 1
                if avposCount > avposStuck:
                    print('Awaiting valid position...', avposCount, avposStuck)
                
                rate.sleep()
                
            avposCount = 0
            
            calcPhotoPosition(photoLat, photoLon, photoBearing, photoAlt)
#            saveFrame = True
#            saveFrameMutex.release()
#            imagingDelay = cameraImageDelay()
#            print('Awaiting [s]: ', imagingDelay)
#            time.sleep(imagingDelay)
                 
        else:
            saveFrameMutex.release()            
            rate.sleep()              

    resetAngleCamera()
    time.sleep(1)


photoLat = 0.0
photoLon = 0.0
photoAlt = 0.0
photoBearing = 0.0
photoPosValid = False
def Camera2StreamCB(img):
    global photoLat, photoLon, photoAlt, photoBearing
    global saveFrame, photoPosValid
    
    saveFrameMutex.acquire()
    if saveFrame:
        saveFrame = False
          
        photoLat = latitude
        photoLon = longitude
        photoAlt = altitude
        photoBearing = bearing
        
        photoPosValid = True
        
        saveFrameMutex.release()
        
        print('Orthocamera - Photo Taken.')
        imgDeepcopy = copy.deepcopy(img)
        saveImage(copy.deepcopy(img))
    else:
        saveFrameMutex.release()
        

    #if rtmpHandler.stream_active:
    #    rtmpHandler.handleFrame(copy.deepcopy(img))


def resetAngleCamera():
    gimbalCmd = GimbalAction()
    gimbalCmdHeader = Header()
    gimbalCmdHeader.frame_id = 'body_FLU'

    gimbalCmd.header = gimbalCmdHeader


    gimbalCmd.is_reset = True
    gimbalCmd.payload_index = 0

    gimbalCmd.rotationMode = 1

    gimbalCmd.pitch = 0.0
    gimbalCmd.roll = 0.0
    gimbalCmd.yaw = 0.0

    gimbalCmd.time = 0.5

    #print('header', gimbalCmd.header)
    #print('is_reset', gimbalCmd.is_reset)
    #print('payload_index', gimbalCmd.payload_index)
    #print('rotationMode', gimbalCmd.rotationMode)

    #print('pitch', gimbalCmd.pitch)
    #print('roll', gimbalCmd.roll)
    #print('yaw', gimbalCmd.yaw)
    #print('time', gimbalCmd.time)
    
    rospy.wait_for_service(dronename+'/gimbal_task_control')
    response = gimbalSrv(gimbalCmd.header, gimbalCmd.is_reset, gimbalCmd.payload_index, gimbalCmd.rotationMode,
                         gimbalCmd.pitch, gimbalCmd.roll, gimbalCmd.yaw, gimbalCmd.time)
    
    print('Reset Camera Angle Response:', response)


def angleCameraOrthophotography():
    # args should be ['header', 'is_reset', 'payload_index', 'rotationMode', 'pitch', 'roll', 'yaw', 'time']
    print('Angling camera downwards')
    

    gimbalCmd = GimbalAction()
    gimbalCmdHeader = Header()
    gimbalCmdHeader.frame_id = 'body_FLU'

    gimbalCmd.header = gimbalCmdHeader

    gimbalCmd.is_reset = False
    gimbalCmd.payload_index = 0

    gimbalCmd.rotationMode = 0

    gimbalCmd.pitch = -90.0
    gimbalCmd.roll = 0.0
    gimbalCmd.yaw = 0.0

    gimbalCmd.time = 0.5

    print('header', gimbalCmd.header)
    print('is_reset', gimbalCmd.is_reset)
    print('payload_index', gimbalCmd.payload_index)
    print('rotationMode', gimbalCmd.rotationMode)

    print('pitch', gimbalCmd.pitch)
    print('roll', gimbalCmd.roll)
    print('yaw', gimbalCmd.yaw)
    print('time', gimbalCmd.time)

    print(gimbalCmd)
    
    rospy.wait_for_service(dronename+'/gimbal_task_control')
    response = gimbalSrv(gimbalCmd.header, gimbalCmd.is_reset, gimbalCmd.payload_index, gimbalCmd.rotationMode,
                         gimbalCmd.pitch, gimbalCmd.roll, gimbalCmd.yaw, gimbalCmd.time)
    
    print('Angled Camera Response:', response)
    print('Camera Setup READY')


overlap = 0.2
def cameraImageDelay():
    global altitude, velocity, overlap
    vfov = 53.64

    photo_capture_frequency_if_drone_is_still = 0.5
    print('velocity:', velocity)
    if velocity >= 1:
        distance = 4.0 * altitude * math.tan(math.radians(vfov / 2.0)) * (1 - overlap)
        print('Velocity:', velocity, 'Distance:', distance)

        rateHz = abs(velocity) / distance
    else:
        rateHz = photo_capture_frequency_if_drone_is_still

    rateSecs = 1.0 / rateHz

    if rateSecs > 8.0:
        rateSecs = 8.0
    if rateSecs < 0.5:
        rateSecs = 0.5

    return rateSecs


def calcPhotoPosition(lat, lon, bearing, h):
    #reference https://www.propelleraero.com/blog/ground-sample-distance-gsd-calculate-drone-data/
    bearing = bearing - 90
    
    horFOV = 68.857347
    verFOV = 41.716547
    
    fovH = 68.857347 * math.pi/180 # The angle on the pyramid from the camera to the ground
    fovV = 41.716547 * math.pi/180
    imW = 608  # To mikos (width)  se PIXELS tis vasis tis piramidas
    imH = 342
    
    timeout = 20.0

    rate = rospy.Rate(30)

    dw = 2 * h * abs(math.tan(fovH/2))      #To width se metra tis vasis tis piramidas pou sximatizete
    dh = 2 * h * abs(math.tan(fovV/2))      #To height se metra tis vasis tis piramidas pou sximatizete
    d = math.sqrt(dw ** 2 + dh ** 2)        # Pithagorio theorima gia na vroume tin diagonio tis photografias se metra
    d = d /1000     #converting ti diagonio se km
    d = d/2     #pianoume ti misi diagonio
    a = 90 - math.atan(imW/imH) * (180/math.pi) #i gonia tou pou vlepei to drone me ti gonia tis photografias
    origin = Point(lat, lon) #The center of the image in lat long form. This is the starting point
    
    #destination1 = geodesic(kilometers=d).destination(origin, bearing + a) # Panw deksia Dias tou tin apostasi kai tin gonia se sxesi me ton vorra, kai sou dinei to simio tou destination se lat long
    #destination2 = geodesic(kilometers=d).destination(origin, bearing + 180 + a) #Katw aristera
    #destination3 = geodesic(kilometers=d).destination(origin, bearing - a) #Panw aristera
    #destination4 = geodesic(kilometers=d).destination(origin, bearing - 180 - a) #katw deksia
    
    topRight = geodesic(kilometers=d).destination(origin, bearing + a)          #Panw deksia Dias tou tin apostasi kai tin gonia se sxesi me ton vorra, kai sou dinei to simio tou destination se lat long
    topLeft = geodesic(kilometers=d).destination(origin, bearing - a)           #Panw aristera
    
    bottomRight = geodesic(kilometers=d).destination(origin, bearing - 180 - a) #katw deksia
    bottomLeft = geodesic(kilometers=d).destination(origin, bearing + 180 + a)  #Katw aristera
    
    # Anamesa Panw Aristera kai Panw Deksia
    
    bottomCenter = Point((bottomLeft.latitude + bottomRight.latitude) / 2, (bottomLeft.longitude + bottomRight.longitude) / 2)
    topCenter = Point((topLeft.latitude + topRight.latitude) / 2, (topLeft.longitude + topRight.longitude) / 2)
    
    center = Point((bottomCenter.latitude + topCenter.latitude) / 2, (bottomCenter.longitude + topCenter.longitude) / 2)
    
    bottom2centerLen = geodesic(bottomCenter, center).m
    #bottom2centerLen = dh
    bottom2topLen = geodesic(bottomCenter, topCenter).m
    print('bottom2centerLen:', bottom2centerLen, 'bottom2topLen:', bottom2topLen)
    center2targetLen = bottom2centerLen * horFOV / 100 * (2 - overlap * 2)
    
    dronePos = Point(latitude, longitude)
    drone2center = geodesic(origin, dronePos).m 
    
    print('drone2center:', drone2center)
    whileFlag = False
    
    
    startTime = time.time()
    printstep = 100
    step = printstep
    
    while center2targetLen > drone2center and orthophotographyActive and time.time() - startTime < timeout:
        if step >= printstep:
            print(str(drone2center) + '/' + str(center2targetLen), str(time.time() - startTime) + '/' + str(timeout))
            step = 0
        whileFlag = True
        dronePos = Point(latitude, longitude)
        drone2center = geodesic(origin, dronePos).m
        rate.sleep()
        step += 1
        
    if not whileFlag:
        print('Did not enter whileFlag')
        
    
    #print('destination5:', destination5.longitude, destination5.latitude)
    #print('initialDist2Destination5:', initialDist2Destination5)
    #print('targetDist2Destination5:', targetDist2Destination5)
    
    #photoPoint = geodesic(meters=targetDist2Drone).destination(origin, bearing)
# =============================================================================
#     distance = geodesic(dronePos, photoPoint).m
#     
#     #print('photoPoint:', photoPoint)
#     #print('distance2photoPoint:', distance)
#     
#     print('initialDist2Center:', initialDist2Center, 'targetDist2Center:', targetDist2Center)
#     print('distance:', distance, 'targetDist2Center:', targetDist2Center)
#     while distance < targetDist2Center and orthophotographyActive:
#         dronePos = Point(latitude, longitude)
#         print(str(distance) + '/' + str(targetDist2Center))
#         distance = geodesic(dronePos, center).m
#         
#         rate.sleep()
# =============================================================================
    
    
    
    # print("[", destination1.longitude, ",", destination1.latitude, "],")
    # print("[", destination4.longitude, ",", destination4.latitude, "],")
    # print("[", destination2.longitude, ",", destination2.latitude, "],")
    # print("[", destination3.longitude, ",", destination3.latitude, "],")
    return [topRight, topLeft, bottomRight, bottomLeft]



def cleanImageDump():
    global imgDirectory
    print('Cleaning Image Directory:')
    for file in os.listdir(imgDirectory):
        if file.endswith('.jpeg'):
            print('\t' + file)
            os.remove(imgDirectory + file)

width = 0
height = 0
def saveImage(img):
    global width, height
    global imgDirectory
    print('Saving Image...')

    if not os.path.isdir(imgDirectory):
        os.mkdir(imgDirectory)

    #rawImg = copy.deepcopy(img)
    imgName = str(int(time.time() * 10**10)) + '.jpeg'
    imgFilePath = imgDirectory + imgName

    #print('Got Image:', img)

    print('Converting to cv2')
    #cv2_img = bridge.imgmsg_to_cv2(img, "rgb8")
    #print('Got cv2_img frfr')
    try:
        # Convert your ROS Image message to OpenCV2
        #
        print('Calling bridge')
        #cv2_img = bridge.imgmsg_to_cv2(img, "rgb8")
        #cv2_img = bridge.imgmsg_to_cv2(img, "passthrough")

        #result = call_python_version("2.7", "python3_cv_bridge", "write_to_file",  
        #                     [img, imgFilePath]) 
        frame = img
        if width * height == 0:
            res_info = rospy.wait_for_message(dronename + '/main_camera_stream_resolution', Resolution)
            width = res_info.width
            height = res_info.height
        frame_array = np.frombuffer(frame.data, dtype=np.uint8).reshape((np.int64(height), np.int64(width), 3))  
        frame_dump = frame_array.tobytes()
        cv2.imwrite(imgFilePath, frame_array)
        #f = open(imgFilePath, 'wb')
        #f.write(frame_dump)
        
        #print('call_python_version:', result)

        print('Got cv2_img')
        # Save your OpenCV2 image as a jpeg
        #cv2.imwrite(imgFilePath, cv2_img)
        print('Image Saved:', imgFilePath)
        
        #updateImageMetadata(imgFilePath)
        
        postImage(imgName)
    except Exception as e:
        print('Caught Exception:', e)

    except CvBridgeError:
        print('Caught Exception:', CvBridgeError)


def updateImageMetadata(img_path):
    global altitude, latitude, longitude, bearing, imgDirectory, platform_ip, buildMapImgUrl
    global d_roll, d_pitch, d_yaw
    global g_roll, g_pitch, g_yaw
    
    et = exiftool.ExifTool()
    #et.start()    
    
    if et.running:
        #EXIF:GPSAltitudeRef 0       
        #EXIF:GPSAltitude 30
        et.execute('-EXIF:GPSAltitudeRef=0', img_path)
        et.execute('-EXIF:GPSAltitude=' + str(altitude), img_path)
        
        #EXIF:GPSLatitudeRef N
        #EXIF:GPSLatitude 34.67607975
        et.execute('-EXIF:GPSLatitudeRef=N', img_path)
        et.execute('-EXIF:GPSLatitude=' + str(latitude), img_path)        
        
        #EXIF:GPSLongitudeRef E
        #EXIF:GPSLongitude 33.0499686111
        et.execute('-EXIF:GPSLongitudeRef=E', img_path)
        et.execute('-EXIF:GPSLongitude=' + str(longitude), img_path)
        
        #XMP:FlightYawDegree -18.0
        et.execute('-XMP:FlightYawDegree=' + str(d_yaw), img_path)
        
        #XMP:FlightPitchDegree: -0.6
        et.execute('-XMP:FlightPitchDegree=' + str(d_pitch), img_path)
        
         #u'XMP:FlightRollDegree': -11.1
        et.execute('-XMP:FlightRollDegree=' + str(d_roll), img_path)
        
        
        
        #XMP:GimbalPitchDegree -48.1
        et.execute('-XMP:GimbalPitchDegree=' + str(g_pitch), img_path)
        
        #XMP:GimbalRollDegree +0.00
        et.execute('-XMP:GimbalRollDegree=' + str(g_roll), img_path)
        
        #XMP:GimbalYawDegree +0.00
        et.execute('-XMP:GimbalYawDegree=' + str(g_yaw), img_path)
        
        
    et.terminate()
    
#    img = pilImage.open(img_path)
#    
#    print('img:', img)
#    
#    exif_dict = piexif.load(img.info["exif"])
#    
#    print('exif_dict:', exif_dict)
    
#    with open(img_path, 'rb') as img_file:
#        img = exifImage(img_file)
#        
#        if img.has_exif:
#            print('Has EXIF')
#        
#        img.bearing = bearing
#        
#        img.altitude = altitude
#        img.latitude = latitude
#        img.longitude = longitude
#        
#        img.d_roll = d_roll
#        img.d_pitch = d_pitch
#        img.d_yaw = d_yaw
#        
#        img.g_roll = g_roll
#        img.g_pitch = g_pitch
#        img.g_yaw = g_yaw
        
        

    


def postImage(imgName):
    global altitude, latitude, longitude, bearing, imgDirectory, platform_ip, buildMapImgUrl
    global d_roll, d_pitch, d_yaw
    global g_roll, g_pitch, g_yaw

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

    #print('Image Full Directory:', myfile)

    files = {'image_file': open(myfile, 'rb')}
    
    if platform_ip:
        if not buildMapImgUrl:
            buildMapImgUrl = 'http://' + platform_ip + ':' + buildMapImgPort + '/postBuildMapImg/'       
            #print ('buildMapImgUrl:', buildMapImgUrl)      
        elif platform_ip not in buildMapImgUrl:
            buildMapImgUrl = 'http://' + platform_ip + ':' + buildMapImgPort + '/postBuildMapImg/'       
            #print ('buildMapImgUrl:', buildMapImgUrl)
    
        try:
            print('buildMapImgUrl:', buildMapImgUrl)
            requests.post(buildMapImgUrl, data=payload, files=files)
            print('POST DONE NP')
        except Exception as e:
            print('Post Image to URL', e)
    else:
        print('No valid platform IP')

    print('Posting Image', imgName, 'done.')


def listener(dji_name = "/matrice300"):
    global gimbalSrv, platform_ip, buildMapImgPort, buildMapImgUrl
    global dronename
    dronename = dji_name
    
    nodename = dronename.replace('/', '') + '_oc'    
    print('Camera Handler', nodename)

    
    rospy.init_node(nodename)
 
    rospy.Subscriber(dronename+'/Telemetry', Telemetry, telemetryCB)
    rospy.Subscriber(dronename+'/BuildMapRequest', BuildMap, cameraStateCB)

    rospy.Subscriber(dronename+'/attitude', QuaternionStamped, DroneOrientationCB)

    rospy.Subscriber(dronename+'/gimbal_angle',Vector3Stamped, GimbalCB)
    gimbalSrv=rospy.ServiceProxy(dronename+'/gimbal_task_control', GimbalAction)

    #rtmpHandler.listener(dronename)
    atexit.register(exit_handler)
    rospy.Subscriber(dronename+'/main_camera/270p30fps/stream', Image, Camera2StreamCB)
    
    
    
    #rospy.Subscriber(dronename+'/main_camera_stream_resolution', Resolution, streamResolutionCB)
    #rospy.Subscriber(dronename+'/main_camera_photo_resolution', Resolution, photoResolutionCB)
    
    # Platform Ip
    rospy.Subscriber(dronename+'/platform/Platform_IP', String, platformIpCB)
    
    print('orthocamera:\t\tREADY')  
    print('Camera Setup READY')
    
    #rospy.spin()
    
    #resetAngleCamera()

    #cameraThread = threading.Thread(target=orthophotography)
    #cameraThread.start()
    
    rospy.spin()


def exit_handler():
    #rtmpHandler.deinitStream()
    pass


if __name__ == '__main__':
    global dronename
    
    print('Initializing Camera Handler...')
    f = open("/var/lib/dbus/machine-id", "r")
    boardId = f.read().strip()[0:4]
    
    dronename = 'matrice300' + '_' + boardId
    
    listener(dronename)
