""""
Video detection using Opencv library
Rafael Makrigiorgis - makrigiorgis.rafael@ucy.ac.cy - 2020
Christos Georgiades - cgeorg15@ucy.ac.cy - 2023
"""

import os, sys
import datetime
import atexit

import math
import cv2

# cwd = os.path.dirname(os.path.realpath(__file__))
# module_dir = cwd + '/detector_utils'
# print('cwd:', cwd)
# print('module_dir:', module_dir)
# print('sys.path:', sys.path)
# #sys.path.insert(0, module_dir)
# sys.path.append(module_dir)
# print('sys.path:', sys.path)

#from src.detector import *
#from src.kalman import *

#from src.trackerkdtree import *
from detector_utils.queue_thread import *
import detector_utils.aruqr_annotation_utils as qrau
#from src.CRPS import *

import threading

import rospy

from std_msgs.msg import Header, String
from std_msgs.msg import Float64MultiArray

from geometry_msgs.msg import Point

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
bridge = CvBridge()


from dji_sdk.msg import Resolution, AruQrDetection

from kios.msg import Telemetry


###############################################
## Global for both RGB VIDEO detection
###############################################

script_dir = os.path.realpath(__file__).replace('/cone_detection.py', '')
print('=*= qr_detection.py =*=')
print('Script Directory:\t' + str(script_dir))

print('cv2 Version:', cv2.__version__)

#video_filename = 'Videos'+'/DJI_0692.MP4'
video_filename = "rtmp://192.168.10.44/live/" #
pathabs = '/home/jetson/swarmonboardpathplanning/src/dji_sdk/scripts/dronepathestimation/CRPS/ekf_withvision/Nov_20_code/'
resize = True
# if you want to resize for better performance
save_results = True # False if you don't want to save detection/results in video/image format
im_width = 1280
im_height = 720
# you need to set the cfg/weights/class files for the NN detections
#config =  '/home/jetson-rps/catkin_ws/CRPS/Python/Configs/anti_drone2.cfg'
# config =  '/home/jetson-rps/catkin_ws/DRPS/Python/new_detection_weights/uav.cfg'
config =  script_dir + '/new_detection_weights/uav.cfg'
cfg_size = (416,416)
cfg_size = (512,512) # mixalis suggestion
#weights = '/home/jetson-rps/catkin_ws/CRPS/Python/Configs/anti_drone2.weights'
# weights = '/home/jetson-rps/catkin_ws/DRPS/Python/new_detection_weights/uav_best.weights'
weights = script_dir + '/new_detection_weights/uav_best.weights'
#classes_file = '/home/jetson-rps/catkin_ws/CRPS/Python/Configs/'+'anti_drone.names'
# classes_file = '/home/jetson-rps/catkin_ws/DRPS/Python/new_detection_weights/uav.names'
classes_file = script_dir + '/new_detection_weights/uav.names'
#print('Config File:\n' + str(config))
#print('Weights File:\n' + str(weights))
#print('Classes File:\n' + str(classes_file) + '\n')
iou_thresh = 0.3
conf_thresh = 0.3
nms_thresh = 0.2
use_torch = False
fps = 0
rpsXY = []
rpsgpsXY=[]
# if use_torch:
#     detectNet = detector_torch('Configs/py_tyv4/best_tiny_yolov4_400ep.pt',
#                                'Configs/py_tyv4/yolov4-tiny', classes=classes_file, imgsz=512,
#                                conf_thresh=conf_thresh, device = 0 ,nms_thresh=nms_thresh)
# else:
#     detectNet = detector(weights, config, conf_thresh=conf_thresh, netsize=cfg_size, nms_thresh=nms_thresh, gpu=True, classes_file=classes_file)
print('Network initialized!')

def rpscallback(rpsdata):
    global rpsXY
    rpsXY.append(rpsdata.data)

def rpsgpscallback(rpsgpsdata):
    global rpsgpsXY
    rgps = [rpsgpsdata.latitude ,rpsgpsdata.longitude, rpsgpsdata.heading ]
    # print(rpsgpsdata.latitude)
    rpsgpsXY.append(rgps)


###################################################################
frame_num = 0
print_every = 100
classes = None
vid_out = None
video = None
frameA = None
Tracking=None
num_det=0
#print('empike :P')

# Rolling Average
lz_center_average = (im_width/2,im_height/2)
win_size = 5
alpha = 2 / (win_size + 1)


use_aruco = True
use_rolling_average_lz = True
############Functions##########################

""""
Processing RGB frames
Applying NN Detection of object for each frame of video or image given
"""


"""
Function for  Video Detection - Traffic Monitoring
"""
def main_TM():
    global frameA
    global classes
    global frame_num
    global fps
    global Tracking
    global crps_data
    global rpsXY,rpsgpsXY

    global lz_center_average, win_size, alpha

    # initializing the video
    # video = cv2.VideoCapture(video_filename)
    # fps = video.get(cv2.CAP_PROP_FPS)  # OpenCV2 version 2 used "CV_CAP_PROP_FPS"
    # frame_count = 9999999999# int(video.get(cv2.CAP_PROP_FRAME_COUNT))
    #duration = int(frame_count / fps)
    #minutes = int(duration / 60)
    #seconds = duration % 60
    #print('Total Video duration (M:S) = ' + str(minutes) + ':' + str(seconds))
    frame_num = 0
    key = None
    vid_out = None
    sum_seconds=0
    start_time=time.time()
    
    print("qr_detection.py - main_TM - Start Time:", time.asctime( time.localtime(time.time()) ))

    vs = VideoStream(detectNet=None,resize=resize,size=(im_width,im_height), dronename=dronename, use_aruco=use_aruco).start()
    time.sleep(1.0)
    key = None
    while not rospy.is_shutdown():
        if key == ord('q') or key == 27:
            vs.stop()
            break

        if frame_num == 0:
            print('qr_detection.py - Awaiting frame from VideoStream')   

            resPacket = Resolution()
            resPacket.width = im_width
            resPacket.height = im_height
            detector_resolution_pub.publish(resPacket)

        while(vs.more() and not rospy.is_shutdown()):
            #print('qr_detection - Got frame')
            # torch.cuda.empty_cache()
            start = time.time()
            #cv2.setMouseCallback("Detection", get_mouse_clicks)


            img, detections, delay = vs.read()
            image = img
            image_cp = image.copy()

            retval, decoded_info, points, straight_qrcode = detections
            
            if frame_num % print_every == 0:
                print('qr detector run successfully')

            if retval:
                if frame_num % print_every == 0:
                    print('qr_detection - Got detections')
                #print('lz_center_average will now crash')
                #print('lz_center_average:', lz_center_average[0], lz_center_average[1])
                #sys.stderr.write('\n')

                #print('lz_center_average:', lz_center_average)
                if not use_aruco:
                    landing_zone_points, landing_zone_center = qrau.calculate_landing_zone_points(points)
                    #print('lz_center_average:', lz_center_average)
                    lz_center_average = (alpha * landing_zone_center[0] + (1 - alpha) * lz_center_average[0] , alpha * landing_zone_center[1] + (1 - alpha) * lz_center_average[1])
                    lz_center_average = (int(lz_center_average[0]), int(lz_center_average[1]))
                    if frame_num % print_every == 0:
                        print('lz_center_average:', lz_center_average)
                    img_center = qrau.get_img_center(img)

                    detected_qr_count = len(decoded_info)
                    min_landing_zone_qrs = 0

                    if detected_qr_count >= min_landing_zone_qrs:
                        landing_zone_center_error = qrau.calculate_point_dist_pixels(img_center, landing_zone_center)
                        if frame_num % print_every == 0:
                            print('points size:', len(points))
                            print('landing_zone_center_error:', int(landing_zone_center_error), '[pixels]')

                        publish_qr_detector_results(img_center, landing_zone_center, landing_zone_center_error)

                    # image_annotated = qrau.annotate_detector_img(image_cp, retval, decoded_info, points, landing_zone_points, landing_zone_center, fps) # processing the image
                    # image_annotated = qrau.draw_img_point(image_annotated, lz_center_average, point_color=(173, 216, 230), point_size=5)
                    # image = image_annotated
                else:
                    # print('Got Aruco Detection')
                    # print('retval:', retval)
                    # print('decoded_info:', decoded_info)
                    # print('points size:', len(points))
                    # print('rejected size:', len(straight_qrcode))

                    rejected = straight_qrcode

                    landing_zone_points, landing_zone_center = qrau.calculate_landing_zone_points(points)
                    #print('lz_center_average:', lz_center_average)
                    lz_center_average = (alpha * landing_zone_center[0] + (1 - alpha) * lz_center_average[0] , alpha * landing_zone_center[1] + (1 - alpha) * lz_center_average[1])
                    lz_center_average = (int(lz_center_average[0]), int(lz_center_average[1]))
                    if frame_num % print_every == 0:
                        print('lz_center_average - aruco:', lz_center_average)

                    img_center = qrau.get_img_center(img)

                    detected_qr_count = len(decoded_info)
                    min_landing_zone_qrs = 0

                    if detected_qr_count >= min_landing_zone_qrs:
                        if use_rolling_average_lz:
                            landing_zone_center_error = qrau.calculate_point_dist_pixels(img_center, lz_center_average)
                            publish_qr_detector_results(img_center, lz_center_average, landing_zone_center_error)
                        else:
                            landing_zone_center_error = qrau.calculate_point_dist_pixels(img_center, landing_zone_center)
                            publish_qr_detector_results(img_center, landing_zone_center, landing_zone_center_error)

                        if frame_num % print_every == 0:
                            print('points size:', len(points))
                            print('landing_zone_center_error:', int(landing_zone_center_error), '[pixels]')

                image_annotated = qrau.annotate_detector_img(image_cp, retval, decoded_info, points, landing_zone_points, landing_zone_center, fps, use_aruco=use_aruco) # processing the image
                image_annotated = qrau.draw_img_point(image_annotated, lz_center_average, point_color=(173, 216, 230), point_size=5)
                image = image_annotated



            if image is not None:
                #print('qr_detection - image is not None')
                #print('decoded_info:', decoded_info)
                if retval:
                    num_det = len(decoded_info)
                    if num_det > 0:
                        if frame_num % print_every == 0:
                            print('Number of detections:', num_det)
                            print('decoded_info:', decoded_info)
                if frame_num == 0:
                    filename =  'qrd_video'#video_filename.split('.')[-2].split('/')[-1]
                else:
                    # updating the tracker using detections and the image frame.
                    test1 = time.time()


                Width = image.shape[1]
                Height = image.shape[0]
                # showing the output image
                font_size = image.shape[0] / 2000
                # cv2.putText(image,'Total Cars: '+str(num_det),(int(Width*0.8),int(Height*0.9)), cv2.FONT_HERSHEY_DUPLEX, font_size, (0.0,0.0,255.0), 1)
                if save_results:
                    # print('qr_detection - saving_results')
                    if frame_num == 0:
                        # creating output folder if save_results is on

                        #vid_fps=video.get(cv2.CAP_PROP_FPS)
                        vid_fps = 24
                        #videoname = video_filename.split('.')[-2].split('/')[-1]

                        path = "Detections/videos/"
                        path = flightlog_path
                        try:
                            if not os.path.exists(path):
                                os.makedirs(path, 0o666)
                        except OSError:
                            print("Creation of the directory %s failed" % path)
                        else:
                            print("Successfully created the directory %s " % path)

                        # initialize output video, using  24 fps
                        #vid_out = cv2.VideoWriter(path + 'det_' + videoname + '.avi', cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), vid_fps, (Width, Height))
                        vid_out = cv2.VideoWriter(flightlog_path + 'matrice300_' + boardId + '-' + initTime + '-aruqr_stream.avi', cv2.VideoWriter_fourcc('A','V','C','1'), vid_fps, (Width, Height))

                    # print('Writing frame to output video')
                    vid_out.write(image) # exporting the image frame to the video



                # cv2.imshow('Detection', image)  # showing a lower resolution image in order to fit the screen.
                #print('qr_detection - calling imshow')
                cv2.imshow('Detection', cv2.resize(image,(1280,720)))  # showing a lower resolution image in order to fit the screen.
                publish_frame(image)
                #print('qr_detection - imshow done\n')

                frame_num = frame_num + 1

                # End time
                end = time.time()

                # Time elapsed
                seconds = end - start
                # Calculate frames per second
                seconds = delay if delay>seconds else seconds

                fps = 1 / seconds
                sum_seconds = sum_seconds + fps
                avg_fps = sum_seconds /frame_num
                if frame_num > 0 and frame_num % 100 == 0:
                    print("Frame: %d  \t FPS/avg %.2f / %.2f \tTotal Dets: %d"%(frame_num,round(fps,2),round(avg_fps,2),num_det))
                    print("Tracking time: %.4f" % (time.time() - test1))
                    print("Delay time: %.4f" % (delay))

                    resPacket = Resolution()
                    resPacket.width = im_width
                    resPacket.height = im_height
                    detector_resolution_pub.publish(resPacket)

                key = cv2.waitKey(1) & 0xFF


            # else:
            #     print('Got None Image')

            # If 'q' or 'Esc' keys are pressed, exit the program
            if key == ord('q') or key == 27:
                break

    print("Turning off camera/video.")
    # video.release()
    print("Ending Time:", time.asctime( time.localtime(time.time()) ))
    print("Duration:",round(time.time()-start_time),"seconds")
    if save_results:
        print("Realising video output.")
        if vid_out:
            vid_out.release()
    print("Camera/Video off.")
    print("qr_detection.py - Program ended.")
    cv2.destroyAllWindows()
    exit(0)


def publish_frame(image):
    global detector_stream_pub

    img_packet = Image()

    imgarray = bridge.cv2_to_imgmsg(image, encoding="passthrough")

    img_packet.encoding = "rgb8"
    img_packet.data = imgarray
    img_packet.header.stamp = rospy.get_rostime()
    img_packet.header.frame_id = "MAIN_CAMERA"

    detector_stream_pub.publish(img_packet)



##############################################################################################
# geometry_msgs/Point img_center
# geometry_msgs/Point landing_zone_center
# float32 landing_zone_error
def update_rolling_avg_pos():
    pass


def publish_qr_detector_results(img_center, landing_zone_center, landing_zone_center_error):
    global aruqr_detector_pub

    img_center_point = Point()
    img_center_point.x = img_center[0]
    img_center_point.z = img_center[1]
    #print('Point created')

    landing_zone_center_point = Point()
    landing_zone_center_point.x = landing_zone_center[0]
    landing_zone_center_point.z = landing_zone_center[1]
    #print('Point created')

    qrd_packet = AruQrDetection()
    #print('QrDetection created')
    qrd_packet.header = Header()
    qrd_packet.img_center = img_center_point
    qrd_packet.landing_zone_center = landing_zone_center_point
    qrd_packet.landing_zone_error = landing_zone_center_error
    qrd_packet.using_aruco = use_aruco
    #print('QrDetection done')

    #print('aruqr_detector_pub publish')
    aruqr_detector_pub.publish(qrd_packet)
    #print('aruqr_detector_pub publish done')


def listener(dji_name = "matrice300"):
    global dronename
    global boardId
    global crps_data
    global detector_resolution_pub, detector_stream_pub
    global aruqr_detector_pub
    global flightlog_path, initTime


    dronename = dji_name
    boardId = dji_name.split('_', 1)[1]
    nodename = dronename.replace('/', '') + '_aruqr_detection'


    initTime = str(datetime.datetime.now().strftime("%Y-%m-%d-%H:%M"))

    try:
        print('qr_detection.py - Listener initialization')
        rospy.init_node(nodename, anonymous=False)
        
        #ekfSub = rospy.Subscriber(dronename + '/EKF', Float64MultiArray, rpscallback)
        #ekfSub_gps = rospy.Subscriber(dronename + '/Telemetry', Telemetry, rpsgpscallback)

        detector_resolution_pub = rospy.Publisher(dronename+ '/aruqr/detector_stream_resolution', Resolution, queue_size=1, latch=True)
        detector_stream_pub = rospy.Publisher(dronename+ '/aruqr/detector_stream', Image, queue_size=1, latch=True)

        aruqr_detector_pub = rospy.Publisher(dronename+ '/aruqr/detections', AruQrDetection, queue_size=1, latch=True)
        

        #"""CRPS"""
        #crps_data = CRPS(dronename=dronename)
    except rospy.ROSInterruptException:
        print('empike mes to exception:P')
        pass
    
    flightlog_path_pckt = rospy.wait_for_message(dronename + '/FlightLogDirectory', String)
    flightlog_path = flightlog_path_pckt.data

    print('Starting qr_detection')
    if __name__ == '__main__':
        main_TM()
    else:
        start_detector_thread()

    


def start_detector_thread():
    global detector_thread

    print('Starting detector_thread...')
    detector_thread = threading.Thread(target=main_TM)
    detector_thread.start()



if __name__ == '__main__':
    print('qr_detection.py starting as independent script')
    #print('Initializing Database Handler...')
    f = open("/var/lib/dbus/machine-id", "r")
    boardId = f.read().strip()[0:4]
    
    dronename = '/matrice300' + '_' + boardId
    listener(dronename)
else:
    print('qr_detection.py starting as import')

    f = open("/var/lib/dbus/machine-id", "r")
    boardId = f.read().strip()[0:4]
    
    dronename = '/matrice300' + '_' + boardId
    listener(dronename)
