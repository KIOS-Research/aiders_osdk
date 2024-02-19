from pprint import pprint

import numpy as np
import cv2
from multiprocessing.pool import ThreadPool
import sys
import queue
from numpy.linalg import inv
import math
import csv

import requests
from geojson import MultiPoint

from src.detector import *
from src.kalman import *
from scipy import signal
import operator
from scipy.interpolate import *
import time
import itertools
import multiprocessing
#import src.CRPS


###########Classes################################

class CRP(object):
    def __init__(self,rpX,rpY,distRef):
        self.rpX = rpX
        self.rpY = rpY
        self.distRef = distRef


class box_in(object):
    def __init__(self, x=None, y=None, w=None, h=None,confidence=None,label = None,direction="",CRP=[]):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.confidence = confidence
        self.label = label
        self.direction = direction
        self.CRP = CRP


class bount_boxes(object):
    def __init__(self,box=None, count_id=None, frame_id=None, kalman=None, kalman_box=None, velocity=0):
        self.box = box
        self.count_id=count_id
        self.frame_id = frame_id
        self.kalman = kalman
        self.kalman_box = kalman_box
        self.active = True
        self.velocity = velocity
        self.lines_before   = [0,0,0,0,0,0,0,0,0]
        self.lines_after    = [0,0,0,0,0,0,0,0,0]
        self.gps_loc = []

"""
Tracking Class
Needs the previous boxes(cars), current detections and image
For first initialization, previous boxes should be an empty array
    Cars => array :
        box         => (x,y,w,h,confidence,label,direction)
        count_id    => int
        frame_id    => array
        kalman      => kalman class
        kalman_box  => (x,y) array
        active      => Boolean
        velocity    => Array
"""
class tracker():
    def __init__(self,  detections, image, draw_track=True,classes=[],iou_thresh=0.3,export=True,filename='',fps=0,droneid=''):
        self.cars = []
        self.detections = detections
        self.image = image
        self.draw_track = draw_track
        self.frame_id=0
        self.classes=classes
        self.iou_thresh=iou_thresh
        self.export=export
        self.filename= filename #.split('.')[0] #remove 'test' and place this one
        self.fps = fps
        # line counter initializations
        self.lines = []
        ow , oh= 3840, 2160 # original width ,original height
        ch,cw = self.image.shape[:2]
        self.trajectories = []
        self.counters = []
        self.droneid = droneid


        # first time initialization
        imH,imW= self.image.shape[:2] # height and width of the image
        if self.image is not None and detections is not None:
            indices, boxes, classIDs, confidences= self.detections
            index = 0
            """ If exporting data is set to true
                Here you can initialize any csv file you will need to create """
            # if (self.export):
            #
            #     fullpath= 'CSV/'+self.filename+'_'
            #     with open(fullpath + 'gps_locs.csv', newline='', mode='w') as csv_file:
            #         fieldnames3 = ['frame', 'timestamp', 'veh_id', 'bboxX', 'bboxY', 'lat', 'long', 'distance']
            #         writergpsloc = csv.DictWriter(csv_file, fieldnames=fieldnames3)
            #         writergpsloc.writeheader()
            # write vehicles csv
            #     with open(fullpath+'vehicles.csv', newline='', mode='w') as csv_file:
            #         fieldnames = ['veh_id', 'video', 'class', 'initW', 'initH', 'initFrame']
            #         writerVeh = csv.DictWriter(csv_file, fieldnames=fieldnames)
            #         writerVeh.writeheader()
            #
            #     with open(fullpath+'tracks.csv', newline='', mode='w') as csv_file:
            #         fieldnames2 = ['veh_id', 'video', 'frame', 'bboxX', 'bboxY', 'Width', 'Height','Direction',
            #                        'Velocity']
            #         writerTra = csv.DictWriter(csv_file, fieldnames=fieldnames2)
            #         writerTra.writeheader()
            #
            #     with open(fullpath + 'lines_init.csv', newline='', mode='w') as csv_file:
            #         fieldnames = ['line_id', 'bbx1', 'bby1', 'bbx2', 'bby2', 'frame']
            #         writerlinit = csv.DictWriter(csv_file, fieldnames=fieldnames)
            #         writerlinit.writeheader()
            #
            #     with open(fullpath + 'lines_track.csv', newline='', mode='w') as csv_file:
            #         fieldnames = ['line_id', 'veh_id', 'type','frame', 'total']
            #         writerltrack = csv.DictWriter(csv_file, fieldnames=fieldnames)
            #         writerltrack.writeheader()

            for i in indices: # loop all detections
                i = i[0]
                box = boxes[i]
                x,y,w,h = box[0],box[1],box[2],box[3]

                box_temp,classid = [round(x), round(y), round(w), round(h)],classIDs[i]
                classid = classIDs[i]
                labelz = str(self.classes[classid])
                confidence=confidences[i]

                temp_box,temp_frame,tmp_kmlbox,tmp_velo = [],[],[],[]

                # checking if the box is out of bounds of the image frame
                x,y,w,h=out_of_bounds((x,y,w,h),0,0,imW,imH)
                temp_box.append(box_in(round(x), round(y), round(w), round(h), confidence, labelz))

                # saving Frame number
                temp_frame.append(self.frame_id)

                # initializing the Kalman filter using the first detection
                XY = (x, y)
                tmp_kalman = KalmanFilter(XY)
                tmp_kmlbox.append(tmp_kalman.predict())
                # Initializing/saving the car
                self.cars.append(bount_boxes(temp_box, index, temp_frame, tmp_kalman, tmp_kmlbox))

                # Initializing the Velocity, it is set as 0 for the first time.
                tmp_velo.append(0)
                self.cars[index].velocity = tmp_velo
                CRP_tmp = []
                #elX, relY, distRef = calc_distance_CRP((imW, imH), self.frame_id, self.cars[index])
              #  CRP_tmp.append(CRP(relX, relY, distRef))
               # self.cars[index].CRP = CRP_tmp

                """ Here you can append to those files, since you already have the results of the first detection"""
                if (self.export):
                # write vehicles csv
                #     fullpath= 'CSV/'+self.filename+'_'
                #     """Getting data from drone and calculating distance and lat long of the detected objects"""
                #     ts, lat, long, alt, head = "2020-09-08 12:22:12", 35.14579902,33.4152465,5.5,-38.20000076# timestamp, lat long heading YOU MUST OBTAIN THIS FROM DRONE
                #     dist_gps, lat2, long2 = self.calc_dist_gps_coords((x, y, w, h), alt, (lat, long), head)
                #     with open(fullpath + 'gps_locs.csv', newline='', mode='a') as csv_file:
                #         fieldnames3 = ['frame', 'timestamp', 'veh_id', 'bboxX', 'bboxY', 'lat', 'long', 'distance']
                #         writergpsloc = csv.DictWriter(csv_file, fieldnames=fieldnames3)
                #         writergpsloc.writerow(
                #             {'frame': self.frame_id, 'timestamp': ts, 'veh_id': index, 'bboxX': x, 'bboxY': y, 'lat': lat2,
                #              'long': long2, 'distance': dist_gps})
                #     temp_gps = self.cars[index].gps_loc
                #     temp_gps.append((dist_gps, lat2, long2))
                #     self.cars[index].gps_loc = temp_gps
                    """old csv data"""
                    # with open(fullpath+'vehicles.csv', newline='', mode='a') as csv_file:
                    #     fieldnames = ['veh_id', 'video', 'class', 'initW', 'initH', 'initFrame']
                    #     writerVeh = csv.DictWriter(csv_file, fieldnames=fieldnames)
                    #     writerVeh.writerow(
                    #         {'veh_id': index, 'video': self.filename, 'class': labelz, 'initW': w, 'initH': h,
                    #          'initFrame': self.frame_id})
                    #
                    # with open(fullpath+'tracks.csv', newline='', mode='a') as csv_file:
                    #     fieldnames2 = ['veh_id', 'video', 'frame', 'bboxX', 'bboxY', 'Width', 'Height', 'Direction',
                    #                    'Velocity']
                    #     writerTra = csv.DictWriter(csv_file, fieldnames=fieldnames2)
                    #     writerTra.writerow(
                    #         {'veh_id': index, 'video': self.filename, 'frame': self.frame_id, 'bboxX': x, 'bboxY': y,
                    #          'Width': w, 'Height': h,'Direction':'','Velocity': 0})

                if self.draw_track:
                    # Displaying the results in the image
                    self.draw_tracks(index)

                # increasing the car index ( car id for tracking it)
                index +=1
        # increasing frame number
        self.frame_id+=1

    """
        Update function of the Tracker, needs to be updated for every frame
    """

    def update(self,detections,image):
        global t0
        self.detections = detections
        self.image = image

        indices, boxes, classIDs, confidences = self.detections
        imH,imW = image.shape[:2]
        adboxes=[]
        detected_objects = []

        """
        Checking all previously active boxes for any match with the current detections
        """
        cars_active = map(lambda car:  car.active == True ,self.cars)
        cars = list(itertools.compress(self.cars,cars_active))
        for car in cars: # getting only the active cars
            found = 0
            # must match it with previous cars
            box_id,box_latest ,kalm_latest= car.count_id,car.box[-1],car.kalman_box[-1]

            box_temp2=[]
            box_temp2 = [kalm_latest[0][0], kalm_latest[1][0], box_latest.w,
                         box_latest.h]

            # finding the iou scores for each detection
            temp_iou = []
            scoreiou=[]

            """iou score"""
            indindex=0
            for i in  indices:
                direction = ""
                i = i[0]
                box = boxes[i]
                x,y,w,h = box[0],box[1],box[2],box[3]

                box_temp = [x, y, w, h]
                classid = classIDs[i]
                labelz = str(self.classes[classid])
                confidence = confidences[i]

                """ [ Hungarian Algorithm ]
                     Checking IOU Threshold between the current detection and the latest kalman detection of each active box.
                     If it's true, the detected object already existed before.
                """
                scoreiou = iou(box_temp2, box_temp)
                if scoreiou >= self.iou_thresh :
                    temp_iou.append((box_temp, indindex, scoreiou, confidence, labelz))
                    found = 1
                    break
                indindex+=1
            """"""

            temp_iou = sorted(temp_iou, key=operator.itemgetter(2), reverse=True)
            if len(temp_iou)>0 :

                x,y,w,h=temp_iou[0][0]
                confidence,labelz = temp_iou[0][3],temp_iou[0][4]
                temp_box,temp_frame = car.box, car.frame_id

                # checking if box is out of bounds
                x, y, w, h = out_of_bounds((x,y,w,h), 0, 0, imW, imH)

                # calculate the direction of the box
                direction = '' #self.get_directions(car.box)
                # appending the info into the corresponding object
                temp_box.append(box_in(round(x), round(y), round(w), round(h), confidence, labelz,direction))
                temp_frame.append(self.frame_id)
                # self.cars[box_id].kalman.predict()
                cm = [[x],[y]]#np.array([[np.float32(x)], [np.float32(y)]])

                # correcting kalman using the detected coordinates
                self.cars[box_id].kalman.correct(cm)

                # doing kalman predict once again in order to be one step ahead
                xykal = self.cars[box_id].kalman.predict()
                kal_tmp = self.cars[box_id].kalman_box
                kal_tmp.append(xykal)

                self.cars[box_id].kalman_box = kal_tmp
                self.cars[box_id].box = temp_box
                self.cars[box_id].frame_id = temp_frame
                index = box_id

                # saving the specific index into an array to prevent duplicates.
                # adboxes.append(temp_scores[0][1])
                indices = np.delete(indices,temp_iou[0][1],axis=0)

                # calculating and saving the velocity
                velocity = 0 #self.calc_velocity(box_id)
                tmp_velo = self.cars[index].velocity
                tmp_velo.append(velocity)
                self.cars[index].velocity = tmp_velo
                font_size = self.image.shape[0] / 3000


                """ Display the results found """
                if self.draw_track:
                    self.draw_tracks(index)
                found = 1 # indicates that a match has been found

                CRP_tmp = self.cars[index].CRP
                #relX, relY, distRef = calc_distance_CRP((imW, imH), self.frame_id, self.cars[index])
                #CRP_tmp.append(CRP(relX, relY, distRef))
               # self.cars[index].CRP = CRP_tmp


                """ Here you can append to those files, since you already have the results of the first detection"""
                # if (self.export):
            if found == 0:
                """ Since it is not found in new detections, checking if it's time to set this as inactive """
                self.check_actives(car.count_id)

            # if found == 0:

        font_size = self.image.shape[0] / 2500
        color = (0.0, 255.0, 255.0)

        """
            Saving all the new detections 
        """

        temp_alt = -1
        for i in indices:# new detections
            # APPEND OBJECTS ON THE ARRAY HERE AS WELL
            i = i[0]
            box = boxes[i]
            x, y, w, h = box[0], box[1], box[2], box[3]

            # self.check_random_spawn(box)
            box_temp = [round(x), round(y), round(w), round(h)]
            classid = classIDs[i]
            labelz = str(self.classes[classid])
            confidence = confidences[i]
            index = len(self.cars)
            temp_box = []
            temp_frame = []
            x, y, w, h = out_of_bounds((x, y, w, h), 0, 0, imW, imH)
            temp_box.append(box_in(round(x), round(y), round(w), round(h), confidence, labelz))
            temp_frame.append(self.frame_id)
            tmp_kalman = []
            tmp_kmlbox = []
            XY = (x, y)
            tmp_kalman = KalmanFilter(XY)
            tmp_kmlbox.append(tmp_kalman.predict())
            self.cars.append(bount_boxes(temp_box, index, temp_frame, tmp_kalman, tmp_kmlbox))
            tmp_velo = []
            tmp_velo.append(0)
            self.cars[index].velocity = tmp_velo

            CRP_tmp = self.cars[index].CRP
            #relX, relY, distRef = calc_distance_CRP((imW, imH), self.frame_id, self.cars[index])
            #CRP_tmp.append(CRP(relX, relY, distRef))
            #self.cars[index].CRP = CRP_tmp

            """ Display the results found """
            if self.draw_track:
                self.draw_tracks(index)
            adboxes.append(i)


        self.frame_id+=1


    """    
    Euclidean distance calculation. 
    """
    def eucl_dist(self,x1,x2,y1,y2):
        result = math.sqrt(pow(abs(x2 - x1), 2.0) + pow(abs(y2 - y1), 2.0))
        return result
    """
    Also using Ground Sample Distance calculation to convert the distance in kilometers / pixel.
    """
    def gsd_calc(self,x1,y1,x2,y2):
        # GSD CALCULATION
        # (FH*SH)/FL*IH)

        # *** Flight Height/Width, Sensor Height, Focal Length, Image Height/Width ***
        # Image Width
        # 1280px
        # Image Height
        # 720 px
        # Sensor Width
        # 6.17mm
        # Sensor Height
        # 4.55 mm
        # Focal Length
        # 5.67 mm or 3.57
        flightH = 20000 #250m = 25000
        sensorH = 0.455
        sensorW = 0.617
        focalL  = 0.567
        imHeight,imWidth = self.image.shape[:2]
        GSDh,GSDw = (flightH*sensorH)/(focalL*imHeight),(flightH*sensorW)/(focalL*imWidth)
        pixels_km= (GSDw/100000) if GSDw > GSDh else (GSDh/100000)  # getting the worst gsd value
        result = math.sqrt(pow(abs(x2 - x1) * pixels_km, 2.0) + pow(abs(y2 - y1) * pixels_km, 2.0))
        return result


    """
     Since it is not found in new detections, checking if it's time to set this as inactive 
    """
    def check_actives(self,index):
        # Set inactive a box if it is not appeared for a few frames. Prevents false box matching.
        box = self.cars[index]
        # If the box is undetected for the past 30 frames, it is inactive.
        if box.frame_id[-1] + 40 <= self.frame_id:
            # bbox_temp = box.box
            # bbox_temp.append(box_in(0, 0, 0, 0))
            self.cars[box.count_id].active = False

        # elif box.frame_id[-1] + 1 <= self.frame_id:
        else:
            """ 
            Kalman Predict so the movement will still be available.
            Also helps for better tracking.
            """

            # kalman
            if len(box.box)>15 :
                x, y, w, h = box.box[-1].x, box.box[-1].y, box.box[-1].w, box.box[-1].h
                sumw,sumh = 0,0

                tmp_box = box.box[-11:]
                sumw,sumh = int(np.sum(list(map(lambda w: w.w, tmp_box)))/len(tmp_box)), \
                            int(np.sum(list(map(lambda h: h.h, tmp_box)))/len(tmp_box))
                # for nn in range(len(box.box)-11,len(box.box)-1):
                #     sumw,sumh = sumw+box.box[nn].w,sumh+box.box[nn].h
                # sumw,sumh = sumw/10,sumh/10
                kx, ky, kw, kh = box.kalman_box[-1][0], box.kalman_box[-1][1],sumw, sumh
                # display kalman filter
                font_size = self.image.shape[0] / 2500

                # cv2.rectangle(self.image, (int(kx), int(ky)), (int(kx + kw), int(ky + kh)), (17.0, 128.0, 213.0), 1)
                # cv2.putText(self.image, 'id'+str(index), (kx - 10, ky - 10), cv2.FONT_HERSHEY_DUPLEX, font_size, (17.0, 128.0, 213.0), 1)

                kal_tmp = self.cars[index].kalman_box
                xykal = self.cars[index].kalman.predict()
                cm = (xykal[0],xykal[1])
                self.cars[index].kalman.correct(cm)

                kal_tmp.append(xykal)

                temp_box = self.cars[index].box
                oldbox = temp_box[-1]
                imH,imW = self.image.shape[:2]
                cm = xykal

                (x,y, kw, kh),out_of = out_of_bounds_actives(( int(cm[0][0]+(kw/2)),  int(cm[1][0]+(kh/2)), kw, kh), 0, 0, imW, imH)
                if out_of == 1:
                    self.cars[box.count_id].active = False
                    return

                temp_box.append(box_in(int(x-(kw/2)),int(y-(kh/2)), kw, kh, 0, oldbox.label, oldbox.direction))

                # saving
                self.cars[index].kalman_box = kal_tmp
                # self.cars[index].box = temp_box

                #save the same frame id
                temp_frame = box.frame_id
                temp_frame.append(self.cars[index].frame_id[-1])
                # self.cars[index].frame_id = temp_frame

                #calculate velocity
                velocity = 0#self.cars[index].velocity[-1]
                tmp_velo = self.cars[index].velocity
                tmp_velo.append(velocity)
                self.cars[index].velocity = tmp_velo



    """ 
    Function used to Display info about the detected objects on the image. (eg bounding box, direction,velocity..)
    """
    def draw_tracks(self,index):

            # getting the correct object based on its index
            box = self.cars[index]
            x,y,w,h,kx,ky,kw,kh = box.box[-1].x,box.box[-1].y,box.box[-1].w,box.box[-1].h,box.kalman_box[-1][0],\
                                  box.kalman_box[-1][1],box.box[-1].w,box.box[-1].h

            index,labelz= box.count_id,box.box[-1].label

            font_size,color,colors = 0.35,(255.0, 0.0, 0.0),[]
            colors.append((255.0, 0.0, 0.0))
            colors.append((0.0, 0.0, 255.0))
            colors.append((0.0, 255.0, 0.0))
            colors.append((0.0, 255.0, 255.0))
            cidx=0 #color index
            if labelz== 'car':
                cidx=0
            elif labelz== 'bus':
                cidx=1
            elif labelz == 'truck':
                cidx = 2
            elif labelz == 'motorbike':
                cidx = 3
            imh,imw = self.image.shape[:2]

            # displaying the bounding box
            cv2.rectangle(self.image, (int(x), int(y)), (int(x + w), int(y + h)),colors[cidx], 1)
            # display kalman filter
            # cv2.rectangle(self.image, (int(kx), int(ky)), (int(kx + kw), int(ky + kh)),(17.0,128.0,213.0), 1)

            # displaying the detected objects class label (eg. car, truck etc..)
            label = labelz + ' #' + str(index)
            cv2.putText(self.image, label, (x, y - int(imh*0.005)), cv2.FONT_HERSHEY_SIMPLEX, font_size, colors[cidx], 1)

            # displaying the direction of the object if it's not empty.
            if box.box[-1].direction is not "":
                #direction = box.box[-1].direction
                cv2.putText(self.image, box.box[-1].direction, (x , y - int(imh*0.015)), cv2.FONT_HERSHEY_SIMPLEX,font_size, color, 1)

            if len(box.gps_loc) > 0:
                dist, lat, long = box.gps_loc[-1]
                cv2.putText(self.image, "dist=%.2f" % dist, (x, y - 25), cv2.FONT_HERSHEY_DUPLEX, font_size,
                            (0.0, 0.0, 255.0), 1)

            #
            # # displaying the velocity of the object if it's not zero.
            # if box.velocity[-1] is not 0:
            #     #velocity = box.velocity[-1]
            #     cv2.putText(self.image, 'vel: '+str( box.velocity[-1]), (x , y - int(imh*0.025)), cv2.FONT_HERSHEY_SIMPLEX, font_size, (180.0,230.0,13.0), 1)



    """"
    Velocity calculation
    Calculating the gsd  between the latest 24 detections.
    Velocity = (Sum of distance) / ( Frame Difference between first and last detection / Frame Rate of the video)
    """
    def calc_velocity(self,index):

        carbox = self.cars[index].box
        car,sum_dist = self.cars[index],0
        # sum_dist = 0
        limit = int(self.fps)

        frame_rate = self.fps*3600 # convert frame rate to hours
        if len(carbox)>= limit:
            frame_diff = self.cars[index].frame_id[-1] - self.cars[index].frame_id[-limit]
            carbox1,carbox2 = carbox[-limit:], carbox[-(limit+1):-1]
            sum = np.sum(list(map(lambda car1,car2: self.gsd_calc(car1.x,car1.y,car2.x,car2.y) ,carbox1,carbox2)))
            # print(sum)
            # for n in range(1,25):
            #     sum_dist = sum_dist + self.gsd_calc(carbox[-n].x,carbox[-n].y,carbox[-n-1].x,carbox[-n-1].y)
        else:
            frame_diff = self.cars[index].frame_id[-1] - self.cars[index].frame_id[-len(carbox)]
            carbox1, carbox2 = carbox[1:],carbox[:-1]
            sum = np.sum(list(map(lambda car1, car2: self.gsd_calc(car1.x, car1.y, car2.x, car2.y), carbox1, carbox2)))
        velo = int((sum) /(frame_diff/frame_rate))
        euc_dist =9999
        #calc dist between last
        if len(carbox) >limit:
            # box2x,box2y=carbox[-5].x,carbox[-5].y
            # box1x,box1y=carbox[-1].x,carbox[-1].y
            # euc_dist = self.eucl_dist(box1x,box2x,box1y,box2y)
            x, y = [], []
            x,y = list(map(lambda x: x.x,carbox[-limit:] )),list(map(lambda x: x.y,carbox[-limit:] ))

            # x = [tmp.x for tmp in carbox[-limit:]]
            # y = [tmp.y for tmp in carbox[-limit:]]

            points = [x, y]
            # times = time.time()
            carbox1 = carbox[1::limit]
            carbox2 = carbox[:-1:limit]
            euc_dist = np.mean(list(map(lambda car1, car2: self.eucl_dist(car1.x, car2.x, car1.y, car2.y), carbox1, carbox2)))
            # print(euc_dist)
            # dists,euc_dist  = normed_distance_along_path(points)
            # print('time',time.time()-times)
        # Set velocity as 0 if it is below 10 and below 7px distance
        if velo <= 15 and euc_dist<15 :
            velo = 0
        return velo

    def draw_count_lines(self):
        for line in self.lines:
            self.image = cv2.line(self.image, (line.xy[0]), line.xy[1], (0,255,0), 1)


    #for line check
    def ccw(self,A, B, C):
        return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

    # Return true if line segments AB and CD intersect
    def intersect(self,A, B, C, D):
        return self.ccw(A, C, D) != self.ccw(B, C, D) and self.ccw(A, B, C) != self.ccw(A, B, D)

    def calc_dist_gps_coords(self, box, altitude, gps, bearing):
        # initially taken from https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=6266436
        # DEPTH AND GEOMETRY FROM A SINGLE 2D IMAGE USING TRIANGULATION , paper
        # camera's parameters
        cam_angl = 45  # camera angle may vary depending on user's input
        # Field of view of the camera of the drone for Mavic enterprise 2
        FOVv = 54.5  # spark
        # FOVv = 55.11 #mavic ent. 2
        FOVh = 69.86  # spark
        # FOVh = 70.57 #mavic ent. 2
        # gps coords
        lat1, long1 = gps
        imh, imw = self.image.shape[:2]
        bbX, bbY, bbW, bbH = box
        bbYl = int(bbY + bbH)
        imH, imW = self.image.shape[:2]
        bearing = -bearing
        frommid = int((bbX + bbW / 2) - (imw / 2))
        if frommid == 0:
            frommid = 1
        if bbYl == 0:
            bbYl = 1
        cntY = imh - bbYl
        if cntY == 0:
            cntY = 1
        afm = -(frommid / imW) * (FOVh / 2)  # angle from middle - X angle
        y = cam_angl + ((imH / 2) - bbYl) * (FOVv / imH)  # ψ angle
        f = (bbX - (imW / 2)) * (FOVh / imW)  # φ angle ( from the center of the image to the ground)
        yrad = math.radians(y)
        frad = math.radians(f)
        gdistY = altitude * (math.tan(yrad))  # Y ground distance to the object
        gdistX = gdistY * (math.tan(frad))  # X ground distance to the object
        bearing2 = bearing + f  # bearing minus the angle of the object from the center

        if bearing2 > 0:
            bearing2 = bearing2 + 180

        print(bearing, bearing2, yrad, frad, gdistY, y, f)
        lat2, long2 = self.pointRadialDistance(lat1, long1, (bearing2), gdistY)
        # print("TEST",box,altitude,gps,bearing,lat2,long2,gdistY,y)
        return gdistY, lat2, long2

    def deg2rad(self, angle):
        return angle * math.pi / 180

    def rad2deg(self, angle):
        return angle * 180 / math.pi

    def pointRadialDistance(self, lat1, lon1, bearing, distance):
        """
        Return final coordinates (lat2,lon2) [in degrees] given initial coordinates
        (lat1,lon1) [in degrees] and a bearing [in degrees] and distance [in km]
        """
        rEarth = 6371.01  # Earth's average radius in km
        epsilon = 0.000001  # threshold for floating-point equality
        rlat1 = math.radians(lat1)
        rlon1 = math.radians(lon1)
        rbearing = math.radians(bearing)
        rdistance = (distance / 1000) / rEarth  # normalize linear distance to radian angle
        rlat = math.asin(
            math.sin(rlat1) * math.cos(rdistance) + math.cos(rlat1) * math.sin(rdistance) * math.cos(rbearing))
        if math.cos(rlat) == 0 or abs(math.cos(rlat)) < epsilon:  # Endpoint a pole
            rlon = rlon1
        else:
            rlon = ((rlon1 - math.asin(math.sin(rbearing) * math.sin(rdistance) / math.cos(rlat)) + math.pi) % (
                    2 * math.pi)) - math.pi
        lat = math.degrees(rlat)
        lon = math.degrees(rlon)
        return (lat, lon)


def iou(bbox1, bbox2):
    """
    Calculates the intersection-over-union of two bounding boxes.
    Args:
        bbox1 (numpy.array, list of floats): bounding box in format x,y,w,h.
        bbox2 (numpy.array, list of floats): bounding box in format x,y,w,h.
    Returns:
        int: intersection-over-onion of bbox1, bbox2
    """
    # bbox1 = [float(x) for x in bbox1]
    # bbox2 = [float(x) for x in bbox2]
    if len(bbox1)==0 or len(bbox2)==0:
        return 0
    (x0_1, y0_1, w1_1, h1_1), (x0_2, y0_2, w1_2, h1_2) = bbox1,bbox2
    # (x0_2, y0_2, w1_2, h1_2) = bbox2
    x1_1,x1_2,y1_1,y1_2 = x0_1 + w1_1,x0_2 + w1_2,y0_1 + h1_1,y0_2 + h1_2
    # x1_2 = x0_2 + w1_2
    # y1_1 = y0_1 + h1_1
    # y1_2 = y0_2 + h1_2
    # get the overlap rectangle
    overlap_x0,overlap_y0,overlap_x1,overlap_y1 = max(x0_1, x0_2),max(y0_1, y0_2),min(x1_1, x1_2),min(y1_1, y1_2)
    # overlap_y0 = max(y0_1, y0_2)
    # overlap_x1 = min(x1_1, x1_2)
    # overlap_y1 = min(y1_1, y1_2)

    # check if there is an overlap
    if overlap_x1 - overlap_x0 <= 0 or overlap_y1 - overlap_y0 <= 0:
        return 0

    # if yes, calculate the ratio of the overlap to each ROI size and the unified size
    size_1, size_2 = (x1_1 - x0_1) * (y1_1 - y0_1),(x1_2 - x0_2) * (y1_2 - y0_2)
    # size_2 = (x1_2 - x0_2) * (y1_2 - y0_2)
    size_intersection = (overlap_x1 - overlap_x0) * (overlap_y1 - overlap_y0)
    size_union = size_1 + size_2 - size_intersection

    # print("intersection-over-onion: " + str(size_intersection / size_union))

    return size_intersection / size_union

""" 
    Checking if the box given is out of the bounds of the image.
    zx,zy must be given as the lowest values (eg. 0,0) and W,H as the maximum Width,Height of the image respectively.
"""
def out_of_bounds(box,zx,zy,W,H):
    x,y,w2,h2 = box
    oobX,oobY = 0,0

    if x < zx:
        x,oobX  = zx,1
        # oobX =1

    elif x>W:
        x,oobX = W, 1
        # oobX =1
    if y < zy:
        y,oobY = zy,1
        # oobY =1

    elif y>H:
        y,oobY = H,1
        # oobY =1

    box = x,y,w2,h2
    if zx ==0 and zy ==0 and oobX ==1 and oobY==1:
        box = 0,0,0,0

    return box
def out_of_bounds_actives(box,zx,zy,W,H):
    x,y,w2,h2 = box
    oobX,oobY = 0,0
    if x < zx:
        x, oobX = zx, 1
        # oobX =1

    elif x > W:
        x, oobX = W, 1
        # oobX =1
    if y < zy:
        y, oobY = zy, 1
        # oobY =1

    elif y > H:
        y, oobY = H, 1
        # oobY =1

    out_of = 0
    box = x,y,w2,h2
    if zx ==0 and zy ==0 and oobX ==1 and oobY==1:
        box = 0,0,0,0
    if  oobX == 1 or oobY == 1:
        out_of = 1
    return box,out_of


def normed_distance_along_path(polyline):
    polyline = np.asarray(polyline)
    distance = np.cumsum(np.sqrt(np.sum(np.diff(polyline, axis=1) ** 2, axis=0)))
    if distance[-1]==0:
        distance[-1]=1

    return np.insert(distance, 0, 0) / distance[-1],distance.mean()

def average_distance_between_polylines(xy1, xy2):
    s1,d1 = normed_distance_along_path(xy1)
    s2,d2 = normed_distance_along_path(xy2)

    interpol_xy1 = interp1d(s1, xy1)
    xy1_on_2 = interpol_xy1(s2)
    node_to_node_distance = np.sqrt(np.sum((xy1_on_2 - xy2) ** 2, axis=0))
    node_to_node_distance2 = [x for x in node_to_node_distance.tolist() if str(x) != 'nan']
    if len(node_to_node_distance2)>0:
        return xy1_on_2,max(node_to_node_distance2)  # or use the max
    else:

        return xy1_on_2,0  # or use the max
