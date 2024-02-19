import rospy
import time

from dji_sdk.srv import SetupCameraStream, Stereo240pSubscription, StereoVGASubscription, StereoDepthSubscription
from sensor_msgs.msg import Image

import threading

import cv2
#from cv_bridge import CvBridge
import numpy as np


#  setup_camera_stream_server_ = nh_.advertiseService(publishName+"/setup_camera_stream", &VehicleNode::setupCameraStreamCallback, this);
#  setup_camera_h264_server_ = nh_.advertiseService(publishName+"/setup_camera_h264", &VehicleNode::setupCameraH264Callback, this);
#  subscribe_stereo_240p_server_  = nh_.advertiseService(publishName+"/stereo_240p_subscription",   &VehicleNode::stereo240pSubscriptionCallback, this);
#  subscribe_stereo_depth_server_ = nh_.advertiseService(publishName+"/stereo_depth_subscription",  &VehicleNode::stereoDepthSubscriptionCallback,this);
#  subscribe_stereo_vga_server_   = nh_.advertiseService(publishName+"/stereo_vga_subscription",    &VehicleNode::stereoVGASubscriptionCallback,  this);


# # assign value to 1 to subscribe
# uint8 front_right_240p
# uint8 front_left_240p
# uint8 down_front_240p
# uint8 down_back_240p
# # if unsubscribe_240p is 1,
# # service will unsubscribe no matter what
# uint8 unsubscribe_240p
def subscribeTo240p(front_right_240p=False, front_left_240p=False, down_front_240p=False, down_back_240p=False, unsubscribe_240p=False):
    time.sleep(2)

    srvPckt = Stereo240pSubscription()
    srvPckt.front_right_240p = int(front_right_240p)
    srvPckt.front_left_240p = int(front_left_240p)
    srvPckt.down_front_240p = int(down_front_240p)
    srvPckt.down_back_240p = int(down_back_240p)
    
    srvPckt.unsubscribe_240p = int(unsubscribe_240p)

    print("stereoSubSrv_request:", int(front_right_240p), int(front_left_240p), int(down_front_240p), int(down_back_240p), int(unsubscribe_240p))
    response = stereoSubSrv(int(front_right_240p), int(front_left_240p), int(down_front_240p), int(down_back_240p), int(unsubscribe_240p))
    #response = stereoSubSrv(srvPckt)
    print("stereoSubSrv_response:", response)

    return response


# ---
# bool result
# StereoVGASubscription
#constant for vga image frequency
# uint8 VGA_20_HZ = 0
# uint8 VGA_10_HZ = 1
# # use above constants to config freq.
# uint8 vga_freq
# # assign value to 1 to subscribe
# uint8 front_vga
# # if unsubscribe_vga is 1,
# # service will unsubscribe no matter what
# uint8 unsubscribe_vga
# ---
# bool result
def subscribeToVGAStereo(VGA_10_HZ=False, unsubscribe=False):
    #time.sleep(2)

    srvPckt = StereoVGASubscription()
    if VGA_10_HZ:
        srvPckt.vga_freq = 1
    else:
        srvPckt.vga_freq = 0

    if not unsubscribe:
        srvPckt.front_vga = 1
        srvPckt.unsubscribe_vga = 0
    else:
        srvPckt.front_vga = 0
        srvPckt.unsubscribe_vga = 1
    print("StereoVGASubscription_request:", 'vga_freq:', int(srvPckt.vga_freq), 'front_vga:', int(srvPckt.front_vga), 'unsubscribe_vga:', int(srvPckt.unsubscribe_vga))
    
    response = stereoSubSrv(int(srvPckt.vga_freq), int(srvPckt.front_vga), int(srvPckt.unsubscribe_vga))
    print("stereoSubSrv_response:", response)

    return response


def unsubscribeToVGAStereo():
    #time.sleep(2)

    srvPckt = StereoVGASubscription()
    srvPckt.vga_freq = 0
    srvPckt.front_vga = 0
    srvPckt.unsubscribe_vga = 1
    print("StereoVGASubscription_request:", 'vga_freq:', int(srvPckt.vga_freq), 'front_vga:', int(srvPckt.front_vga), 'unsubscribe_vga:', int(srvPckt.unsubscribe_vga))
    
    response = stereoSubSrv(int(srvPckt.vga_freq), int(srvPckt.front_vga), int(srvPckt.unsubscribe_vga))
    print("stereoSubSrv_response:", response)

    return response


# SetupCameraStream
# constant for vga image frequency
# uint8 FPV_CAM  = 0
# uint8 MAIN_CAM = 1
# # use above constants to config freq.
# uint8 cameraType
# # 1 for start camera stream, 0 for stop
# uint8 start
# ---
# bool result
def setup_fpv_stream(cameraType, start):
    global fpvSubSrv

    print('Setting up FPV camera')
    srvPckt = SetupCameraStream()
    srvPckt.cameraType = 0
    srvPckt.start = 1
    print("setup_camera_stream_request:", int(srvPckt.cameraType), int(srvPckt.start))
    
    response = fpvSubSrv(int(srvPckt.cameraType), int(srvPckt.start))
    print("setup_camera_stream_response:", response)

    return response


# assign value to 1 to subscribe
# uint8 front_depth_240p
# if unsubscribe_240p is 1,
# service will unsubscribe no matter what
# uint8 unsubscribe_240p
def setup_stereo_depth_stream(front_depth_240p, unsubscribe_240p):
    global stereoDepthSubSrv

    print('Setting up stereo_depth camera')
    print("setup stereo_depth stream request:", front_depth_240p, unsubscribe_240p)
    
    response = stereoDepthSubSrv(front_depth_240p, unsubscribe_240p)
    print("setup stereo_depth stream response:", response)

    return response


def ros_img_2_cv2(frame, layers):
    print('size:', frame.width, 'x', frame.height, 'encoding:', frame.encoding)

    frame_array = np.frombuffer(frame.data, dtype=np.uint8).reshape((np.int64(frame.height), np.int64(frame.width), layers))
    frame_array = cv2.cvtColor(frame_array, cv2.COLOR_RGB2BGR)  
    #frame_dump = frame_array.tobytes()

    return frame_array


def display_stream_worker(stream_topic):
    print('display_stream_worker started')
    #bridge = CvBridge()
    while not rospy.is_shutdown():
        frame = rospy.wait_for_message(stream_topic, Image, timeout=5)
        #cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        
        if frame:
            layers = (int)(frame.step / frame.width)
            if frame.header.frame_id != "Left":
                frame_array = ros_img_2_cv2(frame, layers)
                
                if frame_array.any():
                    print('displaying frame_array')
                    #cv2.imshow('FPV', cv2.resize(frame_array,(1280,720)))
                    cv2.imshow(stream_topic, frame_array)             
                    cv2.waitKey(1)
            else:
                pass


def init():
    global stereo240pSubSrv, stereoSubSrv, fpvSubSrv, stereoDepthSubSrv

    rospy.init_node(nodename, anonymous=False)

    #print("stereoSubSrv subscribing to:", dronename+'/stereo_240p_subscription')
    #stereoSubSrv=rospy.ServiceProxy(dronename+'/stereo_240p_subscription', Stereo240pSubscription)

    # print("\nstereoSubSrv Sleeping")
    # time.sleep(10)
    # unsubscribeToVGAStereo()
    # print("unsubscribeToVGAStereo")
    # print("stereoSubSrv Sleeping")
    # time.sleep(5)
    # subscribeToVGAStereo(VGA_10_HZ=True)

    print("stereoSubSrv subscribing to:", dronename+'/stereo_vga_subscription')
    
    stereo240pSubSrv=rospy.ServiceProxy(dronename+'/stereo_240p_subscription', Stereo240pSubscription)
    stereoSubSrv=rospy.ServiceProxy(dronename+'/stereo_vga_subscription', StereoVGASubscription)
    
    #fpv
    fpvSubSrv=rospy.ServiceProxy(dronename+'/setup_camera_stream', SetupCameraStream)
    #stereo_depth
    stereoDepthSubSrv=rospy.ServiceProxy(dronename+'/stereo_depth_subscription', StereoDepthSubscription)

    

    setup_fpv = True
    if setup_fpv:
        setup_fpv_stream(0, 1)
        x = threading.Thread(target=display_stream_worker, args=(dronename+'/fpv_camera_images',))
        #x.start()

    setup_stereo_depth = False
    if setup_stereo_depth:
        setup_stereo_depth_stream(1, 0)
        x = threading.Thread(target=display_stream_worker, args=(dronename+'/stereo_depth_images',))
        x.start()

    setup_main_camera = False
    if setup_main_camera:
        #setup_stereo_depth_stream(1, 0)
        x = threading.Thread(target=display_stream_worker, args=(dronename+'/main_camera_images',))
        x.start()



def listener(dji_name = "/matrice300", as_module=False):
    global fpvSubSrv, stereoSubSrv, stereoDepthSubSrv
    global dronename, nodename
    dronename = dji_name
    nodename = dronename.replace('/', '') + '_stereo_handler'
    print('Stereo Handler', nodename)
    
    if not as_module:        
        init()
    else:
        print(nodename,'- Imported as a module. Call init when ready to initialize rostopics...')

    if not as_module:
        rospy.spin()


print('Initializing Integrated Camera Handler')
f = open("/var/lib/dbus/machine-id", "r")
boardId = f.read().strip()[0:4]

dronename = '/matrice300' + '_' + boardId

if __name__ == '__main__':
    as_module = False
else:
    as_module = True

listener(dji_name=dronename, as_module=as_module)
