# import the necessary packages
from threading import Thread
import sys
import cv2
import time
# from  src.detect_torch import *
from  src.detector import *
# import the Queue class from Python 3
from queue import Queue
# from src.ros_images import *
import rospy
from sensor_msgs.msg import Image # to be changed?!
from PIL import Image as PilImage
from numba import jit,cuda,njit
#
# imagefeed = []


#

# def get_ros_img():

# rospy.spin()

#@staticmethod
#@jit(nopython=True)
def image_convert(image,height,width):
    # test = np.frombuffer(image,dtype=np.uint8)
    # return test
    return(np.frombuffer(image,dtype=np.uint8).reshape((np.int64(height), np.int64(width), 3)))
    # return np.array(image, dtype=np.uint8)#.reshape((np.int64(height), np.int64(width), 3))

#try:
    #rospy.init_node('visionfusionmode3002', anonymous=False)
	
#except rospy.ROSInterruptException:
#    pass

class VideoStream:
    def __init__(self,  transform=None, queue_size=512,detectNet=None,resize=True,size=(512,512),skip=5, dronename='matrice300'):
        # initialize the file video stream along with the boolean
        # used to indicate if the thread should be stopped or not
        # self.stream = Thread(target=get_ros_img, args=())
        self.stopped = False
        self.transform = transform
        self.detectNet=detectNet
        # initialize the queue used to store frames read from
        # the video file
        self.Q = Queue(maxsize=queue_size)
        # intialize thread
        self.thread = Thread(target=self.update, args=())

        # self.thread.daemon = True
        self.resize=resize
        self.size=size
        self.frameGL = None
        self.detsGL = None
        self.delay=0.0
        self.imgID = 1
        self.skip = skip
        self.tries = 0
        self.callbackframes = 0
        self.streamframes = 0

        #Multimaster
        self.dronename = dronename
        try:
            self.boardId = self.dronename.split('_', 1)[1]
        except Exception as e:
            print('queue_thread - Caught exception:\n', e)
            print('Probably no boardid given')

    def start(self):
        # start a thread to read frames from the file video stream
        self.thread.start()


        # self.thread.join()
        # self.stream.start()

        # rospy.spin()
        return self

    def imagecallback(self,image):
        if self.skip < 4:
            self.skip += 1
            return
        else:
            self.skip = 0
             # calldelay = time.time()
            # print(image.data)
            # Mat mat(img.height, img.width, CV_8UC3, img.rawData.data(), img.width * 3);
            imagetmp = image_convert(image.data,image.height, image.width)
            # imagetmp = np.array(list(image.data), dtype=np.uint8).reshape((image.height, image.width, 3))
            # print(imagetmp)
            # b,g,r = cv2.split(imagetmp)
            # imagefeed = cv2.merge((cv2.split(imagetmp)))
            # imagefeed= cv2.cvtColor(imagetmp, cv2.COLOR_RGB2BGR)
            # imagefeed = cv2.imdecode(np.fromstring(image.data,dtype=np.uint8),cv2.COLOR_RGB2BGR)
            # print(imagefeed)
            # imagefeed = cv2.resiz e(imagefeed, (800,450))
            self.frameGL = imagetmp.copy()
            self.callbackframes +=1
        # print("callback delay",time.time()-calldelay)
        # cv2.waitKey(1)
    def update(self):
        # keep looping infinitely
        global imagefeed
        ekfSub = rospy.Subscriber(self.dronename+'/main_camera_images', Image,
                                  self.imagecallback)  # names will chagne accordingly
        framenum = 0
        self.skip = 0

        while not rospy.is_shutdown():
            # if the thread indicator variable is set, stop the
            # thread
            if self.stopped:
                break

            # otherwise, ensure the queue has room in it
            # if not self.Q.full():
                # print("test m")
                # read the next frame from the file
            skipped = 0
            # print("stream frames", self.streamframes,self.callbackframes)
            # if self.delay >= 0.15 :
            #     self.skip = 10
            # else:
            #     self.skip = 5
            # #
            if self.streamframes != self.callbackframes:
                self.streamframes=self.callbackframes

                    # while skipped != self.skip:
                #     self.skip+=1
                #     time.sleep(0.01)

                #     # if (not grabbed):
                    #    self.tries += 1
                    #    if self.tries == 50:
                    #        self.stopped = True
                    #        break
                # cv2.imshow('test',imgtemp)
                # cv2.waitKey(0)
                # if the `grabbed` boolean is `False`, then we have
                # reached the end of the video file
                if  self.frameGL is not None:
                    self.tries = 0
                    # frame = cv2.resize(self.frameGL, (self.size[0], self.size[1])) if self.resize else self.frameGL
                    self.imgID = self.imgID + 1
                    #
                    try:
                        test = time.time()
                        frame = self.frameGL.copy()
                        dets = self.detectNet.detect(frame)
                        #print('queue_thread - Detections found:', dets)
                        self.delay = time.time() - test
                        #self.frameGL = frame
                        self.detsGL = dets
                        self.Q.put((self.frameGL, self.detsGL, self.delay))


                    except Exception as e:
                        print('error re', e)
                # if self.transform:
                #     frame = self.transform(frame)

                # add the frame to the queue

                # self.Q.put((self.frame,self.dets,self.delay))

                # self.Q.put((frame,dets,delay,(api data))
            # if there are transforms to be done, might as well
            # do them on producer thread before handing back to
            # consumer thread. ie. Usually the producer is so far
            # ahead of consumer that we have time to spare.
            #
            # Python is not parallel but the transform operations
            # are usually OpenCV native so release the GIL.
            #
            # Really just trying to avoid spinning up additional
            # native threads and overheads of additional
            # producer/consumer queues since this one was generally
            # idle grabbing frames.

            # else:
            #     time.sleep(0.1)  # Rest for 10ms, we have a full queue

        #self.stream.release()

    def read(self):
        # return next frame in the queue
        # return (self.frameGL,self.detsGL,self.delay)
        return (self.Q.get())

    # Insufficient to have consumer use while(more()) which does
    # not take into account if the producer has reached end of
    # file stream.
    def running(self):
        return self.more() or not self.stopped

    def more(self):
        # return True if there are still frames in the queue. If stream is not stopped, try to wait a moment
        tries = 0

        while self.Q.qsize() == 0 and not self.stopped and tries < 5 and not rospy.is_shutdown():
            time.sleep(0.1)
            # tries += 1
        #
        # if self.frameGL is not None:
        #     return 1
        # else:
        #     return 0
        return not (self.stopped) and self.frameGL is not None

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True
        #self.stream.release()

        # wait until stream resources are released (producer thread might be still grabbing frame)
        self.thread.join()
