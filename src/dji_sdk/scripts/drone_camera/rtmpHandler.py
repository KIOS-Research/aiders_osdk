#!/usr/bin/env python
import os
import sys
import subprocess
import numpy as np
import time
import rospy

from datetime import datetime

from sensor_msgs.msg import Image

from std_msgs.msg import String
from dji_sdk.msg import Resolution

from threading import Lock

#ip = "10.8.0.2"
#ip = "172.20.228.141"
#ip = "172.20.244.137"
#port = "1935"
#rtmp_url = "rtmp://" + ip + ":" + port + "/live/matrice300"
#rtmp_url = "rtmp://10.42.0.1:1935/live/matrice300"
#rtmp_url = "rtmp://10.16.23.6:1935/live/matrice300"
#rtmp_url = "rtmp://172.20.228.141:1935/live/matrice300"

ffmpeg_p = None
stream_active = False

step = 0

rtmp_mutex = Lock() 

frame_width = 1280
frame_height = 720
fps = 30

dronename = None  
platform_ip = None
rtmp_port = '1935'
rtmp_url = None
def platformIpCB(platformIp):
    global platform_ip, buildMapImgPort, buildMapImgUrl
    
    if platformIp.data == 'None':
        platform_ip = None
        print('rtmp - received None platform Ip')
    elif str(platformIp.data) == str(platform_ip): 
        #print('rtmp - received identical platform Ip')
        pass
    else:
        print('rtmp - received new platform Ip', 'old:', platform_ip, 'new:', platformIp.data)
        platform_ip = platformIp.data
        deinitStream()
        initStream(platform_ip, rtmp_port, dronename)


def initStream(ip, port = rtmp_port, dji_name='/matrice300'):
    global rtmp_mutex
    global rtmp_url, frame_width, frame_height, fps, ffmpeg_p, stream_active, platform_ip, rtmp_url
    global dronename
    dronename = dji_name

    if rtmp_mutex.locked():
        print('rtmp_mutex is locked by another device. Will not initStream')
        return stream_active
    else:
        rtmp_mutex.acquire()
    
    if stream_active:
        print('RTMP: stream already active')
        return stream_active
    
    platform_ip = ip
    rtmp_port = port
    
    if platform_ip and rtmp_port:
        rtmp_url = 'rtmp://' + platform_ip + ':' + rtmp_port + '/live' + dronename
    else:
        print('Failed to Initialize rtmp url', 'platform_ip', platform_ip, 'rtmp_port', rtmp_port)
        return stream_active
    
    
    print('Initializing RTMP stream...')
    print('rtmp_url: ', rtmp_url)
    print('frame_width: ', frame_width)
    print('frame_height: ', frame_height)
    print('fps: ', fps)
    
    if platform_ip is None:
        print ('RTMP - Platform Ip: Unavailable')
        return stream_active
    
    print('Checking RTMP Server status')
    p = subprocess.Popen(("nc", "-zvw5", platform_ip, rtmp_port), stderr = subprocess.PIPE)
    status = p.communicate()[1]
    print (status)

    if 'succeeded' in str(status):
        print("RTMP port available")
    else:
        print('RTMP port unavailable')
        print('Sleeping...')
        time.sleep(5)
        print('RTMP restarting')
        initStream(ip, port, dji_name)
        return stream_active
    
    #res_info = rospy.wait_for_message(dronename + '/main_camera_stream_resolution', Resolution)
    #frame_width = res_info.width
    #frame_height = res_info.height
    
    print('RTMP Stream Res: ' + str(frame_width) + 'x' + str(frame_height) + '@' + str(fps) + 'fps')

    # command and params for ffmpeg
    command = ['ffmpeg',
               '-y',
               '-f', 'rawvideo',
               '-vcodec', 'rawvideo',
               #'-vcodec', 'mjpeg'
               '-pix_fmt', 'rgb24',
               '-s', "{}x{}".format(frame_width, frame_height),
               '-r', str(fps),
               '-i', '-',
               '-c:v', 'libx264',
               '-pix_fmt', 'yuv420p',
               '-preset', 'ultrafast',
               '-an',
               '-f', 'flv',
               rtmp_url]
    
    print('RTMP URL:\t', rtmp_url)
    print('RTMP Command:\n\t', command, '\n')
        
    ffmpeg_p = subprocess.Popen(command, stdin=subprocess.PIPE)
    stream_active = True
    print('RTMP stream ready to receive frames')

    rtmp_mutex.release()

    return stream_active
    

    
def deinitStream():
    global stream_active    
    if stream_active:
        print('Closing RTMP stream...')
        ffmpeg_p.kill()
        stream_active = False
        print('RTMP stream closed')
    

def handle_frame(frame):
    global stream_active, step
    if not stream_active:
        #print('Stream NOT active')
        #print('frame.encoding:', frame.height)
        #print('frame.width:', frame.width)        

        #print('frame.encoding:', frame.encoding)       # Encoding of pixels -- channel meaning, ordering, size
        if platform_ip and rtmp_port and dronename:
            print('handle_frame - calling initStream()')
            stream_initialized = initStream(platform_ip, rtmp_port, dronename)
            print('handle_frame - calling initStream() - stream_initialized:', stream_initialized)

            if not stream_initialized:
                return
        else:
            return

        
       
    #print('handleFrame - received frame')
    stepPrint = 30 * 60
    
    try:
        width = frame.width
        height = frame.height

        #print('frame.width:', frame.width)
        #print('frame.height:', frame.height)
        
        poll = ffmpeg_p.poll()      
        if poll:
            print('ffmpeg_p.poll() ERROR:', poll)
            deinitStream()
            sleep(5)
            initStream(platform_ip, rtmp_port, dronename)

        #print('handleFrame - ffmpeg poll passed')
        frame_array = np.frombuffer(frame.data, dtype=np.uint8).reshape((np.int64(width), np.int64(height), 3))  
        frame_dump = frame_array.tobytes()
        ffmpeg_p.stdin.write(frame_dump)
        #print('handleFrame - ffmpeg write')
        
        if step >= stepPrint:
            step = 0
            print(datetime.now().strftime("%Y/%m/%d-%H:%M:%S"), 'ffmpeg Alive')
        else:
            step += 1

        #print('handleFrame - ffmpeg done')
          
    except Exception as e:
        print("RTMP: Stream Broken", e)
        deinitStream()
        initStream(platform_ip, rtmp_port, dronename)
        pass

 
def init():
    global dronename
    #global 
    
    # Platform Ip
    rospy.Subscriber(dronename + '/platform/Platform_IP', String, platformIpCB)

    # Image Source
    #rospy.Subscriber(dronename+'/main_camera/270p30fps/stream', Image, handle_frame)
    rospy.Subscriber(dronename+'/main_camera_images', Image, handle_frame)

    print(nodename,'- Initialized finished')


def listener(dji_name = "/matrice300", as_module=False):
    global dronename, boardId, nodename


    dronename = dji_name
    boardId = dji_name.split('_', 1)[1]

    nodename = dronename.replace('/', '') + '_rtmp_handler'
    #print('JoyController', nodename)


    if not as_module:
        rospy.init_node(nodename)
        init()
    else:
        print(nodename,'- Imported as a module. Call init when ready to initialize rostopics...')

    rospy.spin()


print('Initializing Joy Controller Handler...')
f = open("/var/lib/dbus/machine-id", "r")
boardId = f.read().strip()[0:4]

dronename = '/matrice300' + '_' + boardId
    
if __name__ == '__main__':
    as_module = False
else:
    as_module = True

listener(dji_name=dronename, as_module=as_module)