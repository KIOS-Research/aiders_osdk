""""
Video detection using Opencv library
Rafael Makrigiorgis - makrigiorgis.rafael@ucy.ac.cy - 2020
"""

from src.trackerkdtree import *
import cv2
import os

from src.detector import *
from src.kalman import *

from src.queue_thread import *
from src.CRPS import *
import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String

from kios.msg import Telemetry
from dji_sdk.msg import Resolution

###############################################
## Global for both RGB VIDEO detection
###############################################

script_dir = os.path.realpath(__file__).replace('/cone_detection.py', '')
print('=*= Monitoring.py =*=')
print('Script Directory:\t' + str(script_dir))

print('Monitoring Version:', cv2.__version__)

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
print('Config File:\n' + str(config))
print('Weights File:\n' + str(weights))
print('Classes File:\n' + str(classes_file) + '\n')
iou_thresh = 0.3
conf_thresh = 0.3
nms_thresh = 0.2
use_torch = False
fps = 0
rpsXY = []
rpsgpsXY=[]
if use_torch:
    detectNet = detector_torch('Configs/py_tyv4/best_tiny_yolov4_400ep.pt',
                               'Configs/py_tyv4/yolov4-tiny', classes=classes_file, imgsz=512,
                               conf_thresh=conf_thresh, device = 0 ,nms_thresh=nms_thresh)
else:
    detectNet = detector(weights, config, conf_thresh=conf_thresh, netsize=cfg_size, nms_thresh=nms_thresh, gpu=True, classes_file=classes_file)
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
classes = None
vid_out = None
video = None
frameA = None
Tracking=None
num_det=0
print('empike :P')

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
    print("Monitoring.py - Start Time:", time.asctime( time.localtime(time.time()) ))

    vs = VideoStream(detectNet=detectNet,resize=resize,size=(im_width,im_height), dronename=dronename).start()
    time.sleep(1.0)
    key = None
    while not rospy.is_shutdown():
        if key == ord('q') or key == 27:
            vs.stop()
            break

        if frame_num == 0:
            print('Monitoring.py - Awaiting frame from VideoStream')   

            resPacket = Resolution()
            resPacket.width = im_width
            resPacket.height = im_height
            detector_resolution_pub.publish(resPacket)

        while(vs.more() and not rospy.is_shutdown()):
           # torch.cuda.empty_cache()
            start = time.time()
            #cv2.setMouseCallback("Detection", get_mouse_clicks)


            img, detections, delay = vs.read()

            image = img
            image_cp = image.copy()
            # image = process_frame(frameA) # processing the image
            if (image is not None):
                if len(rpsgpsXY)>0:
                #if len(rpsXY)>0 and len(rpsgpsXY)>0:
                    # crps_data.update(rpsXY[-1],rpsgpsXY[-1])
                    crps_data.update(rpsgpsXY[-1])

                # crps_data.update((20,-3))

                num_det = len(detections[0])
                if frame_num == 0:
                    filename =  'crps_video'#video_filename.split('.')[-2].split('/')[-1]
                    print('Monitoring.py - Initializing tracker...')
                    Tracking = tracker(detections, image, True, detectNet.classes, iou_thresh, export=False,
                                       filename=filename, fps=fps,CRPS_data=crps_data)
                else:
                    # updating the tracker using detections and the image frame.
                    test1 = time.time()
                    if detections is not None:
                        #print('Found Detections:\t', detections)
                        Tracking.update(detections, image,crps_data)
                        if len(detections[1]) > 0:
                            #print('detections:', detections)
                            pass
                            #detection_bounding_box = String()
                            #detection_bounding_box.data = str(detections[1][0])
                            #detector_bounding_box_pub.publish(detection_bounding_box)

                Width = image.shape[1]
                Height = image.shape[0]
                # showing the output image
                font_size = image.shape[0] / 2000
                # cv2.putText(image,'Total Cars: '+str(num_det),(int(Width*0.8),int(Height*0.9)), cv2.FONT_HERSHEY_DUPLEX, font_size, (0.0,0.0,255.0), 1)
                if save_results:
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
                        vid_out = cv2.VideoWriter(flightlog_path + 'matrice300_' + boardId + '-' + initTime + '-cone_stream.avi', cv2.VideoWriter_fourcc('A','V','C','1'), vid_fps, (Width, Height))

                    vid_out.write(image) # exporting the image frame to the video

                # cv2.imshow('Detection', image)  # showing a lower resolution image in order to fit the screen.
                cv2.imshow('Detection', cv2.resize(image,(1280,720)))  # showing a lower resolution image in order to fit the screen.

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
        vid_out.release()
    print("Camera/Video off.")
    print("Program ended.")
    cv2.destroyAllWindows()
    exit(0)

##############################################################################################


def listener(dji_name = "matrice300"):
    global dronename
    global boardId
    global crps_data
    global detector_resolution_pub#, detector_bounding_box_pub
    global flightlog_path, initTime

    dronename = dji_name
    boardId = dji_name.split('_', 1)[1]
    nodename = dronename.replace('/', '') + '_visionfusionmode300'


    initTime = str(datetime.datetime.now().strftime("%Y-%m-%d-%H:%M"))

    try:
        print('Monitoring.py - Listener initialization')
        rospy.init_node(nodename, anonymous=False)
        
        ekfSub = rospy.Subscriber(dronename + '/EKF', Float64MultiArray, rpscallback)
        #ekfSub_gps = rospy.Subscriber(dronename + '/telemetry2', Telemetry, rpsgpscallback)
        ekfSub_gps = rospy.Subscriber(dronename + '/Telemetry', Telemetry, rpsgpscallback)

        detector_resolution_pub = rospy.Publisher(dronename+ '/detector_stream_resolution', Resolution, queue_size=1, latch=True)
        #detector_bounding_box_pub = rospy.Publisher(dronename+ '/detector_bounding_box', String)
        

        """CRPS"""
        crps_data = CRPS(dronename=dronename)
    except rospy.ROSInterruptException:
        print('empike mes to exception:P')
        pass
    
    flightlog_path_pckt = rospy.wait_for_message(dronename + '/FlightLogDirectory', String)
    flightlog_path = flightlog_path_pckt.data

    main_TM()


if __name__ == '__main__':
    print('Monitoring.py starting...')
    #print('Initializing Database Handler...')
    f = open("/var/lib/dbus/machine-id", "r")
    boardId = f.read().strip()[0:4]
    
    dronename = '/matrice300' + '_' + boardId
    listener(dronename)
