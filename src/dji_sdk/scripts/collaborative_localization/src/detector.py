import cv2
import numpy as np
import numba as nb
"""
Darknet yolo detector class using OPENCV DNN library
"""
class detector():

    def __init__(self,weights=None,config=None,netsize=(1024,1024),conf_thresh=0.5,nms_thresh=0.4,gpu=True,classes_file='names.txt'):
        self.weights = weights
        self.config = config
        self.gpu = gpu
        self.net  = cv2.dnn.readNet(weights,config )
        # self.net  = cv2.dnn.readNetFromDarknet(config, weights)
        self.classes_file = classes_file
        if gpu:
            #set CUDA as the preferable backend and target
            print("[INFO] setting preferable backend and target to CUDA...")
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
        else:
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_OPENCL_FP16)



        ln = self.net.getLayerNames()
        #self.layers = [ln[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]
        #self.layers = [ln[i - 1] for i in self.net.getUnconnectedOutLayers()]

        try:
            self.vers = 0
            self.layers = [ln[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]
        except:
            self.vers = 1
            self.layers = [ln[i - 1] for i in self.net.getUnconnectedOutLayers()]

        print('Using self.vers=', self.vers)


        self.netsize = netsize
        self.conf_thresh = conf_thresh
        self.nms_thresh = nms_thresh
        self.detections=[]
        classes = []
        with open(classes_file, 'r') as f:
            classes = [line.strip() for line in f.readlines()]
        self.classes = classes

        self.model = cv2.dnn_DetectionModel(self.net)
        self.model.setInputParams(size=(self.netsize[0], self.netsize[1]), scale=1 / 256)

    def detect(self, frame):
        imH, imW = frame.shape[:2]

        imaget = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # imaget=frame # when capturing rgb
        image = cv2.resize(imaget, (self.netsize[0], self.netsize[1]), interpolation=cv2.INTER_LINEAR)
        #print('Image resize done')

        classes, scores, boxes = [],[],[]
        try:
            # cv2.imshow("test",cv2.cvtColor(image,cv2.COLOR_RGB2BGR))
            # cv2.waitKey(0)
            classes, scores, boxes = self.model.detect(image, self.conf_thresh, self.nms_thresh)
            #print('Detection done')
        except Exception as e:
            print('Exception during detection')
            print(e)

            return [],[],[],[]

        if len(boxes)==0:
            #print('No Detection')
            return [],[],[],[]
        indices = np.arange(0, len(boxes) )

        #print('Sorting into classes and indices')
        try:
            classes = list(map(lambda x: x[0], classes))
            indices = list(map(lambda x: [x], indices))
        except:
            classes = list(map(lambda x: [x], classes))
            indices = list(map(lambda x: [x], indices))
        #print('Sorting Success')
        

        #@staticmethod
        #@nb.jit(nopython=True)
        def convert(x):
            x,y,w,h= int((x[0] / self.netsize[0]) * imW),int((x[1] / self.netsize[1]) * imH), int((x[2] / self.netsize[0]) * imW), \
            int((x[3] / self.netsize[1]) * imH)
            return [x,y,w,h]


        boxes2=list(map(convert ,boxes))
        #print('Detection found. Converting boxes. Returning')
        return indices,boxes2,classes,scores

   
