import numpy as np
from matplotlib import pyplot as plt
import math
import rospy
from kios.msg import Telemetry
from dji_sdk.msg import BoundingBoxes
import os
import time
import datetime
from geopy import distance
from std_msgs.msg import Header, Float64MultiArray, String, Float32

class CRPS:
    def __init__(self,KNOWN_DISTANCE=8.0, KNOWN_WIDTH=0.82, W_pixel=132,rps_data = [], dronename='matrice210v2vis'):
        self.KNOWN_DISTANCE = KNOWN_DISTANCE
        # initialize the known object width
        self.KNOWN_WIDTH = KNOWN_WIDTH
        # initialize the known pixel width #image size 1080p?
        self.W_pixel = W_pixel
        # initialize the self.focalLength
        self.focalLength = (self.W_pixel * self.KNOWN_DISTANCE) / self.KNOWN_WIDTH
        
        self.rpsXY = rps_data
        self.mergeXY = []
        self.distXY = []

        flightlog_path_pckt = rospy.wait_for_message(dronename + '/FlightLogDirectory', String)
        self.flightlog_path = flightlog_path_pckt.data
        self.initTime = str(datetime.datetime.now().strftime("%Y-%m-%d-%H:%M"))

        now = datetime.datetime.now()
        time = now.strftime("%Y%m%d%H%M")
        self.results_path = "results_crps"+time+".txt"
        self.log_crps = False
        # plt.ion()
        # plt.show()
        self.folrpsgps = []
        self.rpsgpsXY = []

        self.dmetersoffsetx = 0
        self.dmetersoffsety = 0
        self.dlattest = 0
        self.dlontest = 0
        self.antihaver = []
        self.bearingrad = 0
        self.dist_eucl = []

                #Multimaster
        self.dronename = dronename
        self.droneId = dronename.split('_')[-1]

        self.zoom_sub = rospy.Subscriber("/matrice300_"+self.droneId+"/main_camera_parameters", Float32, self.cameraZoomParametersCB)
        self.zoom = 1.0
        try:
            self.boardId = self.dronename.split('_', 1)[1]
        except Exception as e:
            print('queue_thread - Caught exception:\n', e)
            print('Probably no boardid given')


        if self.log_crps and not os.path.exists(self.results_path):
            f = open(self.results_path, "w")
            f.write('Frame, RelativeX, RelativeY, X_Lead, Y_Lead, Timestamp, Drone_ID, GPSR_X, GPSR_Y, GPSF_X, GPSF_Y\n')
            f.close()
        try:
            # self.rpsddPub = rospy.Publisher(self.dronename+'/crps/Telemetry', Telemetry, queue_size=1)
            # self.bbPub = rospy.Publisher(self.dronename+'/crps/bounding_boxes', BoundingBoxes, queue_size=1)
            # self.visPub = rospy.Publisher(self.dronename+'/crps/visiondata', Float64MultiArray, queue_size=1)
            self.rpsddPub = rospy.Publisher(self.dronename+'/crps/Telemetry', Telemetry, queue_size=1, latch=True)
            self.bbPub = rospy.Publisher(self.dronename+'/crps/bounding_boxes', BoundingBoxes, queue_size=1)
            self.visPub = rospy.Publisher(self.dronename+'/crps/visiondata', Float64MultiArray, queue_size=1)


        except rospy.ROSInterruptException:
            pass

    def cameraZoomParametersCB(self, zoomPckt):
        self.zoom = zoomPckt.data

    # def rpscallback(self,rpsdata):
    #     print("test callbk")
    #     # glorpsdata.append(rpsdata.data) # floats with all the floats
    #     self.rpsXY.append(rpsdata.data)
    #def update(self,rpsdata,rpsgpsXY):
    def update(self,rpsgpsXY):
        #self.rpsXY.append(rpsdata)
        self.folrpsgps.append(rpsgpsXY)
        if len(self.rpsgpsXY)==0:
            self.rpsgpsXY.append(rpsgpsXY)

    def pointRadialDistance( lat1, lon1, bearing, distance):
        """    Return final coordinates (lat2,lon2) [in degrees] given initial coordinates    (lat1,lon1) [in degrees] and a bearing [in degrees] and distance [in km]    """    
        rEarth = 6378.01  # Earth's average radius in km    
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
            rlon = ((rlon1 + math.asin(math.sin(rbearing) * math.sin(rdistance) / math.cos(rlat)) + math.pi) % (
                2 * math.pi)) - math.pi
        lat = math.degrees(rlat)
        lon = math.degrees(rlon)
        return (lat, lon)

    def calc_lat_long(self):
        #if len(self.mergeXY)>1:
        if len(self.distXY)>1:           
            try:
                # read drone bearing
                bearing = self.folrpsgps[-1][2]# +360 bearing from telemetry ( test +360 if not working properly) 
                bearingY = -(math.atan(self.distXY[-1][0]/self.distXY[-1][1])*(180 / math.pi)) + bearing

                # calculate distance using pythagorean theorem
                a = math.sqrt(self.distXY[-1][0]**2 + self.distXY[-1][1]**2)/1000.0
                # print(a,self.distXY[-1][0], self.distXY[-1][1])
                # convert that distance to an distance object
                d = distance.distance(kilometers=a)
                
                #print('self.folrpsgps[-1]=', self.folrpsgps[-1])

                # convert that distance object into a geopoint 
                uvar = d.destination(point=(self.folrpsgps[-1][0],self.folrpsgps[-1][1]), bearing=bearingY)
                self.rpsgpsXY.append((uvar.latitude,uvar.longitude,bearingY))

                #if tele is not None:
                #drone_id = tele.serialVersionUID.split('/crps')[0].split('_')[1]

                file1 = open(self.flightlog_path + self.droneId + "_monitoring_internal-" + self.initTime +".csv", "a")  # append mode
                file1.write(f"{str(datetime.datetime.now())},{bearing},{bearingY},{self.distXY[-1][0]},{self.distXY[-1][1]},{self.folrpsgps[-1][0]},{self.folrpsgps[-1][1]},{self.rpsgpsXY[-1][0]},{self.rpsgpsXY[-1][1]},{self.zoom}\n")
                file1.close()

                print('Saved ' + self.droneId + ' measurement')

                DDdata = Telemetry()
                DDdata.rostime_secs = int(time.time())
                DDdata.rostime_nsecs = int(time.time() % 1 * 10**9)
                DDdata.latitude = self.rpsgpsXY[-1][0]
                DDdata.longitude = self.rpsgpsXY[-1][1]
                DDdata.heading = self.folrpsgps[-1][2]
                DDdata.serialVersionUID = self.droneId + '_crps'
                self.rpsddPub.publish(DDdata)
            except Exception as e:
                print('CRPS - Caught Exception: ', e)
                print('self.folrpsgps: ', self.folrpsgps)
                print('Is telemetry ok?')
                #print('bearing=', bearing)
                #print('CRPS - Caught Exception: ', e)

    def calc_distance_CRP(self,framesize, frameid, drone):
        # X,Y,W,H = box
        imW, imH = framesize
        F_ID = frameid
        B_ID = drone.count_id
        ref_distance = 0
        boxl = drone.box[-1]
        box = BoundingBoxes()
        box.header = Header()
        box.header.stamp = rospy.get_rostime()
        box.labels.append("drone")
        box.confidences.append(1.0)
        box.tops.append(boxl.x/imW)
        box.lefts.append(boxl.y/imH)
        box.rights.append(boxl.x/imW + boxl.w/imW)
        box.bottoms.append(boxl.y/imH + boxl.h/imH)
        self.bbPub.publish(box)
        # calculate Z from X, Y, Z
        # initialize the known distance from the camera to the object

        Z_MA = []
        X_MA = []
        Y_MA = []
        ID_fltr = B_ID

        # loop over the frames
        i = 0
        #if len(self.rpsXY)==0:
        #return (0,0),(0,0),(0,0)
        distx, disty = [], []
        first = 0

        limit = len(drone.box) if len(drone.box)<=25 else 25 # plus one to get from max to 25 below
        #print(len(drone.box))
        for n in range(len(drone.box)-1,len(drone.box)-limit-1,-1):
            Z_MA.append(self.distance_to_camera(self.KNOWN_WIDTH, drone.box[n].w))
            X_MA.append(drone.box[n].x)
            Y_MA.append(drone.box[n].y)
            length, position = width_of_cam(imW, self.KNOWN_WIDTH, drone.box[n].w, drone.box[n].x)
            distx.append(-(position - length / 2))
            disty.append(Z_MA[-1])
            # print('Img Width %.2f Drone at position %.2f, %.2f meters from center, distance to drone %.2f, frame %d, ID %d '%(length,position,(position-length/2),Z_MA[-1],F_ID[i],B_ID[i]))
            i += 1

        # averaging the results and turning them into the same number of variables as the relative metrics
        count = 0
        sumx = 0
        sumy = 0
        filt_X = []
        filt_Y = []
        # timi = 24
        # if len(distx) < 24:
        #     step = len(distx) - 1
        # else:
        #     step = 23
        for n in range(0, len(distx)):
            sumx = sumx + distx[n]
            sumy = sumy + disty[n]


        filt_X.append(sumx / (len(distx)))  # get the avg of the N frames
        filt_Y.append(sumy / (len(distx)))
 
        self.distXY.append((filt_X[-1],filt_Y[-1]))
        self.dist_eucl.append( math.sqrt(pow(abs(filt_X[-1]), 2.0) + pow(abs(filt_Y[-1]), 2.0)))

        self.calc_lat_long() ### NEW ADDITION 2023
        
        vision_data = Float64MultiArray()
        vision_data.data = [self.dist_eucl[-1],filt_X[-1], filt_Y[-1] ]
        self.visPub.publish(vision_data)
        return  filt_X[-1], filt_Y[-1], self.dist_eucl[-1]

    def distance_to_camera(self,knownWidth, perWidth):
        # compute and return the distance from the maker to the camera
        return (knownWidth * self.focalLength) / perWidth


def haversine(lon1, lat1, lon2, lat2):
    """
    Calculate the great circle distance between two points
    on the earth (specified in decimal degrees)
    """
    # convert decimal degrees to radians
    lon1, lat1, lon2, lat2 = map(math.radians, [lon1, lat1, lon2, lat2])

    # haversine formula
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.asin(math.sqrt(a))
    r = 6371 # Radius of earth in kilometers. Use 3956 for miles
    return c * r


def width_of_cam(imgW,dronesizem,dronesizepx,X):
    # compute and return the distance from the maker to the camera

    # normalize to 1920 (initial img size) that took comparison
    # of the drone size in the first place
    drone_normalize = imgW / 1920

    meters_length = (imgW /(dronesizepx*drone_normalize))* dronesizem
    position = (X/imgW)*meters_length
    return meters_length,position


def update(num, data, line):
    line.set_data(data[0:2, :num])
    line.set_3d_properties(data[2, :num])
    return line

def normalize(X):
    x_max = max(X)
    x_min = min(X)
    s = x_max - x_min
    X_new = []
    for x in X:
        X_new.append((x - x_min) / s)
    return X_new

    # Function to find distance
def distance_f(x1, y1, z1, x2, y2, z2):
    d = math.sqrt(math.pow(x2 - x1, 2) +
                  math.pow(y2 - y1, 2) +
                  math.pow(z2 - z1, 2) * 1.0)
    return d




