//
// Created by dji on 2020/5/8.
//
#include <dji_sdk/dji_vehicle_node.h>
#include <tf/tf.h>
#include <sensor_msgs/Joy.h>
#include <dji_telemetry.hpp>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <stdio.h>
#include <math.h>
#include <cmath>
#include <algorithm>
#include <stdlib.h>
#include <unistd.h>
#include <ctime>

#include "boost/date_time/posix_time/posix_time.hpp"
#include <fstream>

using namespace dji_sdk;

#define _TICK2ROSTIME(tick) (ros::Duration((double)(tick) / 1000.0))
static int constexpr STABLE_ALIGNMENT_COUNT = 400;
static double constexpr TIME_DIFF_CHECK = 0.008;
static double constexpr TIME_DIFF_ALERT = 0.020;


typedef struct box
{
    float x, y, w, h;
} box;

typedef struct box_car
{
    vector<box> b;
    bool 			active;
    int 			id;
    float 			mid_cnt;
    float 			mid_bot;
    int 			weight;
    float 			volume;
    float 			prob;
    vector<int> placement;
} box_car;

int ID = 0;

float confThreshold = 0.25; // Confidence threshold
float nmsThreshold = 0.4;  // Non-maximum suppression threshold
long long int frame_id = 0;
Mat im;

DJI::OSDK::Control::CtrlData flightControlData = DJI::OSDK::Control::CtrlData(0x43, 0.0, 0.0, 0.0, 0.0);
// 0x43 (hex) =    01       00       0          01           1       (binary)
//            = HORI_VEL/VERT_VEL/YAW_ANG/BODY_FRAME/STABLE_MODE
bool hasAuthority=false;

vector<box_car>    cars;
vector<int> 		actives;
vector<vector<long long int>> frame_indicate;
long long int 		frame_num = 0;
static bool findTargetId = true;
int targetId;
int counter;
vector<int> ids;
float locY = -1;
double dt = 0.1; //1.0/30; // Time step
string fileName;
std::string time_str;

vector<int> classIds;
vector<float> confidences;
vector<Rect> boxes2;

void postprocess(Mat frame, const vector<Mat>& outs);
vector<String> getOutputsNames(const Net& net);
//int addBoxDemo(Mat im, float xcenter, float ycenter, float x, float y, float w, float h, int frame, int classes, int classn, float prob);
float overlap(float x1, float w1, float x2, float w2);
float box_intersection(box a, box b);
float box_union(box a, box b);
float box_iou(box a, box b);
void force_zero();
void set_frame_id(long long int frame_id);
double MidPoint(Point2f pt1, Point2f pt2);
int mostFrequent(vector<int> arr);
bool pMode = false;
bool sModeTriggered = false;
bool startTrackingFlag = false;
double areaDiffThresh = 100.0;
box averageTargetBox;

float box_iou(box a, box b)
{
    return box_intersection(a, b)/box_union(a, b);
}

float overlap(float x1, float w1, float x2, float w2)
{
    float l1 = x1 - w1/2;
    float l2 = x2 - w2/2;
    float left = l1 > l2 ? l1 : l2;
    float r1 = x1 + w1/2;
    float r2 = x2 + w2/2;
    float right = r1 < r2 ? r1 : r2;
    return right - left;
}

float box_intersection(box a, box b)
{
    float w = overlap(a.x, a.w, b.x, b.w);
    float h = overlap(a.y, a.h, b.y, b.h);
    if(w < 0 || h < 0) return 0;
    float area = w*h;
    return area;
}

float box_union(box a, box b)
{
    float i = box_intersection(a, b);
    float u = a.w*a.h + b.w*b.h - i;
    return u;
}

//if a box not detected for a certain period of time (50 frames) it will reset to zero
void force_zero()
{
    for(int i=0; i < cars.size();i++)
    {
	//if (!findTargetId && i == targetId)
	//	continue;
        int frame_diff = frame_num - frame_indicate[i][frame_indicate[i].size()-1];
        if (frame_diff>35)
        {
            box zeros;
            zeros.x=0;zeros.y=0;zeros.w=0;zeros.h=0;
            vector<box> emptyB;
            emptyB.push_back(zeros);
            cars[i].b = emptyB;
            cars[i].active  = false;
            cars[i].mid_cnt = 0;
            cars[i].mid_bot = 0;
            cars[i].weight  = 0;
            //            boxes[i]=zeros;
        }
    }
}

void set_frame_id(long long int frame_id)
{
    frame_num = frame_id;
}

//Euclidian distance of 2 points
double MidPoint(Point2f pt1, Point2f pt2)
{
    double result = sqrt(pow(pt2.x - pt1.x, 2.0) + pow(pt2.y - pt1.y, 2.0));
    return result;
}

// Get the names of the output layers
vector<String> getOutputsNames(const Net& net)
{
    static vector<String> names;
    if (names.empty())
    {
        //Get the indices of the output layers, i.e. the layers with unconnected outputs
        vector<int> outLayers = net.getUnconnectedOutLayers();

        //get the names of all the layers in the network
        vector<String> layersNames = net.getLayerNames();

        // Get the names of the output layers in names
        names.resize(outLayers.size());
        for (size_t i = 0; i < outLayers.size(); ++i)
            names[i] = layersNames[outLayers[i] - 1];
    }
    return names;
}

long int kalmanOverDistance=0;
int kalmanOverDistanceThresh=3;
// Remove the bounding boxes with low confidence using non-maxima suppression
void postprocess(Mat frame, const vector<Mat>& outs)
{
    //vector<int> 	classIds;
    vector<float> 	confidences;
    vector<Rect> 	boxes;
    vector<int>     nmsClassIds;
    vector<float>   nmsConfidences;
    vector<Rect>    nmsBoxes;
   
    for (size_t i = 0; i < outs.size(); ++i)
    {
        // Scan through all the bounding boxes output from the network and keep only the
        // ones with high confidence scores. Assign the box's class label as the class
        // with the highest score for the box.
        float* data = (float*)outs[i].data;
        for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
        {
            Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
            Point classIdPoint;
            double confidence;
            // Get the value and location of the maximum score
            minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
            if (confidence > confThreshold)
            {
                int centerX = (int)(data[0] * frame.cols);
                int centerY = (int)(data[1] * frame.rows);
                int width = (int)(data[2] * frame.cols);
                int height = (int)(data[3] * frame.rows);
                int left = centerX - width / 2;
                int top = centerY - height / 2;

                classIds.push_back(classIdPoint.x);
                confidences.push_back((float)confidence);
                boxes.push_back(Rect(left, top, width, height));

            }
        }
    }


    // Perform non maximum suppression to eliminate redundant overlapping boxes with
    // lower confidences
    vector<int> indices;
    Size s = frame.size();
    double distance=sqrt(pow(s.width,2)+pow(s.height,2))*2;
    int distIndex=targetId;
    NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);

    for (size_t i = 0; i < indices.size(); ++i)
    {
        int idx = indices[i];
        nmsClassIds.push_back(classIds[idx]);
        nmsConfidences.push_back(confidences[idx]);
        nmsBoxes.push_back(boxes[idx]);

        int centerX =  (nmsBoxes[i].x + nmsBoxes[i].width/2.);
        int centerY = (nmsBoxes[i].y + nmsBoxes[i].height/2.);
        //        int top =   (nmsBoxes[i].y - nmsBoxes[i].height/2.);
        //        int bot =   (nmsBoxes[i].y + nmsBoxes[i].height/2.);
        int width = nmsBoxes[i].width;
        int height = nmsBoxes[i].height;
        float confidence = confidences[i];

        //        drawPred(classIds[idx], confidences[idx], box.x, box.y, box.x + box.width, box.y + box.height, frame);
	//std::cout << std::endl << "-------- Before addBox --------" << std::endl;
        //int boxID = addBoxDemo(frame, centerX, centerY, centerX, centerY, width, height, 0, 1, 1, confidence);
	//std::cout << std::endl << "-------- After addBox --------" << std::endl;
	//TODO: CHANGE THIS boxID here!!!!!!!!!!!!!!!!!!!!	
	int boxID = 0; 
        if(!findTargetId){
	    	int lastb=cars[targetId].b.size()-1;
		double temp = sqrt(pow((cars[targetId].b[lastb].x - centerX),2) + pow((cars[targetId].b[lastb].y - centerY),2));
		areaDiffThresh = averageTargetBox.w * averageTargetBox.h * 1.0;
 		double areaDiff = abs(averageTargetBox.w * averageTargetBox.h - width*height);
		if((temp <= distance) && (areaDiff <= areaDiffThresh)){
			distance = temp;
			distIndex = boxID;
			//std::cout << std::endl << "-------- new shortest distance to target: " << distance << " with ID: " << distIndex << " --------" << std::endl;
		}
	}
    }


    //std::cout << std::endl << "-------- Actives Size: " << actives.size() << " --------" << std::endl;
    if(!findTargetId && actives.size()>0){
	    //cout<<"targetId: "<<targetId<<" distIndex: "<<distIndex<<endl;
	    int lastb=cars[targetId].b.size()-1;
	    int lastbDist=cars[distIndex].b.size()-1;
	    //cout<<"lastb: "<<lastb<<" lastbDist "<<lastbDist<<endl;
	    //std::cout << std::endl << "-------- Checking for target with distance --------" << std::endl;
	    bool stillActive = false;
            for(int k=0; k < actives.size(); k++){
	    	//std::cout << std::endl << "-------- Checking actives for id: " << k << " --------" << std::endl;
            	if(actives[k] == targetId)
            	    stillActive = true;
            }
	    //std::cout << std::endl << "-------- Target Active?: " << stillActive << " --------" << std::endl;
	    if(!stillActive){
		//std::cout << std::endl << "-------- target max(w,h): " << max(cars[targetId].b[lastb].h, cars[targetId].b[lastb].w) << " --------" << std::endl;
		kalmanOverDistance++;
		if ((distance <= max(averageTargetBox.h, averageTargetBox.w)*1.5) && kalmanOverDistance>kalmanOverDistanceThresh){
		        cars[targetId].b[lastb].x = cars[distIndex].b.at(lastbDist).x;
			cars[targetId].b[lastb].y = cars[distIndex].b.at(lastbDist).y;
			cars[targetId].b[lastb].w = cars[distIndex].b.at(lastbDist).w;
			cars[targetId].b[lastb].h = cars[distIndex].b.at(lastbDist).h;
			cars[targetId].active   = true;
	    		cars[targetId].prob	= cars[distIndex].prob;
			cars[targetId].mid_cnt	= cars[distIndex].mid_cnt;
		    	cars[targetId].mid_bot	= cars[distIndex].mid_bot;
		    	cars[targetId].weight	= cars[distIndex].weight;
			cars[targetId].volume	= cars[distIndex].volume;
			actives.push_back(targetId);

			box zeros;
		    	zeros.x=0;zeros.y=0;zeros.w=0;zeros.h=0;
		    	vector<box> emptyB;
		    	emptyB.push_back(zeros);
		    	cars[distIndex].b = emptyB;
		    	cars[distIndex].active  = false;
		    	cars[distIndex].mid_cnt = 0;
		    	cars[distIndex].mid_bot = 0;
		    	cars[distIndex].weight  = 0;
			//std::cout << std::endl << "--------------------- from new id to target w: " << cars[targetId].b[lastb].w << " h: " << cars[targetId].b[lastb].h << " --------------" << std::endl;
		}		
	    }else{
		//std::cout << std::endl << "--------------------- from IOU to target --------------" << std::endl;
		kalmanOverDistance=0;
	    }
    }

    classIds = nmsClassIds;
    confidences = nmsConfidences;
    //    boxes = nmsBoxes;

    ++frame_id;
    //        std::cout << "Frame: " << frame_num << std::endl;
    set_frame_id(frame_id);
    force_zero();

}

Rect2d bbox;
bool nearestUsed=false;
/*
void VehicleNode::nearestBox(float x, float y)
{
    //if(!nearestUsed){
	    findTargetId=true;
	    Size s = im.size();
	    double minValue = 1465.0;
	    int idx;
	    Point2f target_center(x*s.width, y*s.height); //(s.width)

	    bool useKalman=true;
	    ifstream kalmanFile;
	    kalmanFile.open("/home/jetson/catkin_ws/kalmanOrNot.txt");
	    if(!kalmanFile){
			useKalman=true;
	    }else
	       		kalmanFile >> useKalman;
	    kalmanFile.close();
	    if(useKalman){

		    for(int i=0; i<cars.size();i++)
		    {
			if(cars[i].active){
			    int lastb=cars[i].b.size()-1;
			    cv::Point2f box_center = cv::Point2f(cars[i].b[lastb].x, cars[i].b[lastb].y);
			    double d = MidPoint(target_center, box_center);
			    if (d < minValue && cars[i].b[lastb].x>2 && cars[i].b[lastb].y>2)
			    {
				minValue = d;
				idx = i;
			    }
			}
		    }

		    if (cars.size() > 0)
		    {
			if(findTargetId)
			{
			    std::cout << "Frame: " << frame_num << "\t Searching for the target car!" << std::endl;
			    ids.push_back(cars[idx].id);
			    findTargetId = false;
			    targetId = mostFrequent(ids);
			    std::cout << "\nFrame: " << frame_num << "\t The target id is: " << targetId << std::endl;
			    int lastb=cars[targetId].b.size()-1;
			    locY = cars[targetId].b[lastb].y - cars[targetId].b[lastb].h/2;
			    std::cout << "\nFrame: " << frame_num << "\t The locY is: " << locY << std::endl;
			    // Best guess of initial states
			    Eigen::VectorXd x0(4);
			    x0 << cars[targetId].b[lastb].x, cars[targetId].b[lastb].y, 0.0, 0.0;
			    VehicleNode::getKalmanFilter()->init(0.0, x0);
			    VehicleNode::ptr_wrapper_->stopWaypointV2Mission(1);
			}
			else
			{

			    //std::cout << "\nFrame: " << frame_num<< "\t Draw the target car only" << std::endl;
			    int lastb=cars[targetId].b.size()-1;
			    Scalar color= CV_RGB(0,0,255); // #1 box is blue
			    Point xycntr = Point(cars[targetId].b[lastb].x,cars[targetId].b[lastb].y);
			    Point wh = Point(cars[targetId].b[lastb].w,cars[targetId].b[lastb].h);
			    Point topl = Point(xycntr.x-wh.x/2,xycntr.y-wh.y/2);
			    Point botr = Point(xycntr.x+wh.x/2,xycntr.y+wh.y/2);

			    Point text_lbl = Point(topl.x -5,topl.y -5);
			    //rectangle(im,topl,botr, color,2);
			    string textlbl =to_string(cars[targetId].id);//to_string(cars[actives[idx]].id);
			    //putText(im, textlbl,text_lbl, FONT_HERSHEY_TRIPLEX, 1, Scalar(255,255,255));
			}
		    }
		    else
			std::cout << "cars size is 0!" << std::endl;
	       }else{
			std::cout << std::endl << "------------- INITIALIZING BOX AT x: " << int(x*s.width) << ", y: " << int(y*s.height) << " -------------" << std::endl;
			// Define initial bounding box 
	    	        bbox = Rect2d(int(x*s.width)-40, int(y*s.height)-40, 80, 80);

			std::cout << std::endl << "---------- CLEARING TRACKER! ----------" << std::endl;
			VehicleNode::getTracker()->clear();
                        Ptr<Tracker> trackerNew;

			// Construct the OpenCV tracker
                        int type;
                        ifstream trackerFile;
                        trackerFile.open("/home/jetson/catkin_ws/trackerType.txt");
                        if(!trackerFile){
                            type=2;
                        }else
                            trackerFile >> type;
                        trackerFile.close();
   
                        // List of tracker types in OpenCV 3.4.1
                        string trackerTypes[8] = {"BOOSTING", "MIL", "KCF", "TLD","MEDIANFLOW", "GOTURN", "MOSSE", "CSRT"};
                        string trackerType = trackerTypes[type];

                        if (trackerType == "BOOSTING")
                           trackerNew = TrackerBoosting::create();
                        else if (trackerType == "MIL")
                           trackerNew = TrackerMIL::create();
                        else if (trackerType == "KCF")
                           trackerNew = TrackerKCF::create();
                        else if (trackerType == "TLD")
                           trackerNew = TrackerTLD::create();
                        else if (trackerType == "MEDIANFLOW")
                           trackerNew = TrackerMedianFlow::create();
                        else if (trackerType == "GOTURN")
                           trackerNew = TrackerGOTURN::create();
                        else if (trackerType == "MOSSE")
                           trackerNew = TrackerMOSSE::create();
                        else if (trackerType == "CSRT")
                           trackerNew = TrackerCSRT::create();
			
			VehicleNode::setTracker(trackerNew);

			VehicleNode::getTracker()->init(im, bbox);
			findTargetId=false;
	       }
	  //     nearestUsed=true;
	//}

}
(
int mostFrequent(vector<int> arr)
{
    size_t n = arr.size();
    // Insert all elements in hash.
    unordered_map<int, int> hash;
    for (int i = 0; i < n; i++)
        hash[arr[i]]++;

    // find the max frequency
    int max_count = 0, res = -1;
    for (auto i : hash) {
        if (max_count < i.second) {
            res = i.first;
            max_count = i.second;
        }
    }

    return res;
}

int addBoxDemo(Mat im, float xcenter, float ycenter, float x, float y, float w, float h, int frame, int classes, int classn, float prob)
{
    cv::Size s = im.size();

    int left  = (x-w/2.);
    //    int right = (x+w/2.);
    int top   = (y-h/2.);
    //    int bot   = (y+h/2.);

    //    int offset = classn*123457 % classes;

    float red = 2;
    float green = 1;
    float blue = 0;
    float rgb[3];
    rgb[0] = red;
    rgb[1] = green;
    rgb[2] = blue;

    std::vector<int> p;
    std::vector<float> scores;

    //    std::vector<Point2f> p_lines;
    //    std::vector<Point2f> p_lines2;

    int stored = 0;

    box temp;
    temp.x = x;
    temp.y = y;
    temp.w = w;
    temp.h = h;


    int found = 0;

    if(!findTargetId){
	int lastb=cars[targetId].b.size()-1;
	box b = cars[targetId].b[lastb];
        float score = box_iou(temp,b);
	//std::cout << "==========================" << std::endl << "Box IOU with targetId: " << score << std::endl << "==========================" << std::endl;
        if(score >= 0.20)
        {
            p.push_back(targetId);
            scores.push_back(score);
        };
    }

    // keep track of the same detection box ID
    for(size_t d = 0; d < cars.size(); d++)
    {
        if(cars[d].active == true){
            int lastb=cars[d].b.size()-1;
            box b = cars[d].b[lastb];
            float score = box_iou(temp,b);
            if(score >= 0.20)
            {
                p.push_back(d);
                scores.push_back(score);
            };
        }

    }

    int highestScore=0;
    for (int q=1; q < p.size(); q++){
        if(scores[q] > scores[highestScore])
            highestScore=q;
    }


    if(frame_indicate.size()>0 && p.size() > 0){
        if(frame_indicate[p[highestScore]][frame_indicate[p[highestScore]].size()-1]==frame_num)  //duplicates prevention
        {
            return p[highestScore];
        }
        else
        {
	    if (!findTargetId && (p[highestScore] == targetId) ){
		int lastb=cars[targetId].b.size()-1;
		float widthC = temp.w;
	    	float heightC = temp.h;
		//average the target's box size
	        int numOfBoxes = std::min( 25, (int)cars[targetId].b.size());
	        for(int indx = lastb; indx > cars[targetId].b.size()-numOfBoxes; indx--){
	    		widthC = (widthC + cars[targetId].b[indx].w);
			heightC = (heightC + cars[targetId].b[indx].h);
		}

		averageTargetBox.x = temp.x;
		averageTargetBox.y = temp.y;
		averageTargetBox.w = widthC/numOfBoxes;
		averageTargetBox.h = heightC/numOfBoxes;

            	cars[p[highestScore]].b.push_back(temp);
	    }else{
		// raf : save the latest box if not detected?
	        int lastb=cars[p[highestScore]].b.size()-1;
	        cars[p[highestScore]].b[lastb].x = temp.x;
	        cars[p[highestScore]].b[lastb].y = temp.y;
	        cars[p[highestScore]].b[lastb].w = temp.w;
	        cars[p[highestScore]].b[lastb].h = temp.h;
	    }

            found = 1;
            stored = p[highestScore];
        }
    }

    if(found == 0)
    {
        //        boxes.push_back(temp);
        std::vector<box> newcar;
        newcar.push_back(temp);
        stored=cars.size();

        box_car temp_car;
        temp_car.b = newcar;
        temp_car.active = true;
        temp_car.weight=0;
        temp_car.id=stored;
        temp_car.mid_cnt=0;
        temp_car.mid_bot=0;
        temp_car.volume=0;
        temp_car.prob=prob;
        cars.push_back(temp_car);

        std::vector<long long int> temp_frm;
        temp_frm.push_back(frame_num);
        frame_indicate.push_back(temp_frm);

    }

    Point2f image_bot((s.width / 2),(s.height));

    cars[stored].prob=prob;
    cars[stored].active = true;
    actives.push_back(stored);

    int lastb=cars[stored].b.size()-1;

    // getting midpoints from center and bottom-center for each box
    cv::Point2f box_center = cv::Point2f(cars[stored].b[lastb].x, cars[stored].b[lastb].y);
    //cars[stored].mid_cnt = MidPoint(image_cnt, box_center);
    cars[stored].mid_bot = MidPoint(image_bot, box_center);
    cars[stored].volume  = cars[stored].b[lastb].w; //* (h*s.height); //volume of each box
    rectangle(im,Point(cars[stored].b[lastb].x-cars[stored].b[lastb].w/2,cars[stored].b[lastb].y-cars[stored].b[lastb].h/2),Point(cars[stored].b[lastb].x+cars[stored].b[lastb].w/2,cars[stored].b[lastb].y+cars[stored].b[lastb].h/2), CV_RGB(255,255,255),2);
    frame_indicate[stored].push_back(frame_num); //saving the frame for each box

    Point xycntr = Point(cars[stored].b[lastb].x,cars[stored].b[lastb].y);
    Point wh = Point(cars[stored].b[lastb].w,cars[stored].b[lastb].h);
    Point topl = Point(xycntr.x-wh.x/2,xycntr.y-wh.y/2);

    string textlbl =to_string(cars[stored].id);//to_string(cars[actives[idx]].id);
    Point text_lbl = Point(topl.x -5,topl.y -5);
    putText(im, textlbl,text_lbl, FONT_HERSHEY_TRIPLEX, 1, Scalar(255,255,255));

    return stored;
}
*/

void VehicleNode::SDKBroadcastCallback(Vehicle* vehicle, RecvContainer recvFrame, DJI::OSDK::UserData userData)
{
  ((VehicleNode*)userData)->dataBroadcastCallback();
}

void VehicleNode::dataBroadcastCallback()
{
  using namespace DJI::OSDK;

  ros::Time now_time = ros::Time::now();

  uint16_t data_enable_flag = ptr_wrapper_->getPassFlag();

  uint16_t flag_has_rc = ptr_wrapper_->isM100() ? (data_enable_flag & DataBroadcast::DATA_ENABLE_FLAG::HAS_RC) :
                         (data_enable_flag & DataBroadcast::DATA_ENABLE_FLAG::A3_HAS_RC);
  if (flag_has_rc)
  {
    sensor_msgs::Joy rc_joy;
    rc_joy.header.stamp    = now_time;
    rc_joy.header.frame_id = "rcBroad";

    rc_joy.axes.reserve(6);
    rc_joy.axes.push_back(static_cast<float>(ptr_wrapper_->getRC().roll     / 10000.0));
    rc_joy.axes.push_back(static_cast<float>(ptr_wrapper_->getRC().pitch    / 10000.0));
    rc_joy.axes.push_back(static_cast<float>(ptr_wrapper_->getRC().yaw      / 10000.0));
    rc_joy.axes.push_back(static_cast<float>(ptr_wrapper_->getRC().throttle / 10000.0));

    rc_joy.axes.push_back(static_cast<float>(ptr_wrapper_->getRC().mode));
    rc_joy.axes.push_back(static_cast<float>(ptr_wrapper_->getRC().gear));
    rc_publisher_.publish(rc_joy);
  }

  tf::Matrix3x3 R_FRD2NED;
  tf::Quaternion q_FLU2ENU;

  if (data_enable_flag & DataBroadcast::DATA_ENABLE_FLAG::HAS_Q)
  {
    R_FRD2NED.setRotation(tf::Quaternion(ptr_wrapper_->getQuaternion().q1,
                                         ptr_wrapper_->getQuaternion().q2,
                                         ptr_wrapper_->getQuaternion().q3,
                                         ptr_wrapper_->getQuaternion().q0));
    tf::Matrix3x3 R_FLU2ENU = R_ENU2NED_.transpose() * R_FRD2NED * R_FLU2FRD_;
    R_FLU2ENU.getRotation(q_FLU2ENU);

    geometry_msgs::QuaternionStamped q;
    q.header.stamp = now_time;
    q.header.frame_id = "body_FLU";

    q.quaternion.w = q_FLU2ENU.getW();
    q.quaternion.x = q_FLU2ENU.getX();
    q.quaternion.y = q_FLU2ENU.getY();
    q.quaternion.z = q_FLU2ENU.getZ();

    attitude_publisher_.publish(q);
  }

  if ( (data_enable_flag & DataBroadcast::DATA_ENABLE_FLAG::HAS_Q) &&
       (data_enable_flag & DataBroadcast::DATA_ENABLE_FLAG::HAS_W) &&
       (data_enable_flag & DataBroadcast::DATA_ENABLE_FLAG::HAS_A))
  {
    sensor_msgs::Imu imu;

    imu.header.frame_id = "body_FLU";
    imu.header.stamp    = now_time;

    imu.linear_acceleration.x =  ptr_wrapper_->getAcceleration().x * gravity_const_;
    imu.linear_acceleration.y = -ptr_wrapper_->getAcceleration().y * gravity_const_;
    imu.linear_acceleration.z = -ptr_wrapper_->getAcceleration().z * gravity_const_;

    imu.angular_velocity.x    =  ptr_wrapper_->getAngularRate().x;
    imu.angular_velocity.y    = -ptr_wrapper_->getAngularRate().y;
    imu.angular_velocity.z    = -ptr_wrapper_->getAngularRate().z;

    // Since the orientation is duplicated from attitude
    // at this point, q_FLU2ENU has already been updated
    imu.orientation.w = q_FLU2ENU.getW();
    imu.orientation.x = q_FLU2ENU.getX();
    imu.orientation.y = q_FLU2ENU.getY();
    imu.orientation.z = q_FLU2ENU.getZ();

    imu_publisher_.publish(imu);
  }

  if (data_enable_flag & DataBroadcast::DATA_ENABLE_FLAG::HAS_POS)
  {
    DJI::OSDK::Telemetry::GlobalPosition global_pos = ptr_wrapper_->getGlobalPosition();
    std_msgs::UInt8 gps_health;
    gps_health.data = global_pos.health;
    gps_health_publisher_.publish(gps_health);

    sensor_msgs::NavSatFix gps_pos;
    gps_pos.header.stamp    = now_time;
    gps_pos.header.frame_id = "gps";
    gps_pos.latitude        = global_pos.latitude * 180 / C_PI;
    gps_pos.longitude       = global_pos.longitude * 180 / C_PI;
    gps_pos.altitude        = global_pos.altitude;
    this->current_gps_latitude_ = gps_pos.latitude;
    this->current_gps_longitude_ = gps_pos.longitude;
    this->current_gps_altitude_ = gps_pos.altitude;
    this->current_gps_health_ = global_pos.health;
    gps_position_publisher_.publish(gps_pos);

    if(local_pos_ref_set_)
    {
      geometry_msgs::PointStamped local_pos;
      local_pos.header.frame_id = "/local";
      local_pos.header.stamp = now_time;
      gpsConvertENU(local_pos.point.x, local_pos.point.y, gps_pos.longitude,
                    gps_pos.latitude, this->local_pos_ref_longitude_, this->local_pos_ref_latitude_);
      local_pos.point.z = gps_pos.altitude - this->local_pos_ref_altitude_;
      /*!
      * note: We are now following REP 103 to use ENU for
      *       short-range Cartesian representations. Local position is published
      *       in ENU Frame
      */

      this->local_position_publisher_.publish(local_pos);
    }

    std_msgs::Float32 agl_height;
    agl_height.data = global_pos.height;
    height_publisher_.publish(agl_height);
  }

  if (data_enable_flag & DataBroadcast::DATA_ENABLE_FLAG::HAS_V)
  {
    geometry_msgs::Vector3Stamped velocity;
    velocity.header.stamp    = now_time;
    velocity.header.frame_id = "ground_ENU";

    Telemetry::Vector3f ground_velocity = ptr_wrapper_->getVelocity();
    //std::cout << "ground_velocity.x:" << ground_velocity.x << "\t" << ground_velocity.y << "\t" << ground_velocity.z << "\n";

    velocity.vector.x = ground_velocity.y;
    velocity.vector.y = ground_velocity.x;
    velocity.vector.z = ground_velocity.z;

    velocity_publisher_.publish(velocity);
  }

  uint16_t flag_has_battery =
      ptr_wrapper_->isM100() ? (data_enable_flag & DataBroadcast::DATA_ENABLE_FLAG::HAS_BATTERY) :
      (data_enable_flag & DataBroadcast::DATA_ENABLE_FLAG::A3_HAS_BATTERY);

  if ( flag_has_battery )
  {
    sensor_msgs::BatteryState msg_battery_state;
    msg_battery_state.header.stamp = now_time;
    msg_battery_state.capacity = ptr_wrapper_->getBatteryInfo().capacity;
    msg_battery_state.voltage  = ptr_wrapper_->getBatteryInfo().voltage;
    msg_battery_state.current  = ptr_wrapper_->getBatteryInfo().current;
    msg_battery_state.percentage = ptr_wrapper_->getBatteryInfo().percentage;
    msg_battery_state.charge   = NAN;
    msg_battery_state.design_capacity = NAN;
    msg_battery_state.power_supply_health = msg_battery_state.POWER_SUPPLY_HEALTH_UNKNOWN;
    msg_battery_state.power_supply_status = msg_battery_state.POWER_SUPPLY_STATUS_UNKNOWN;
    msg_battery_state.power_supply_technology = msg_battery_state.POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
    msg_battery_state.present = (ptr_wrapper_->getBatteryInfo().voltage!=0);
    battery_state_publisher_.publish(msg_battery_state);
  }

  uint16_t flag_has_status =
      ptr_wrapper_->isM100() ? (data_enable_flag & DataBroadcast::DATA_ENABLE_FLAG::HAS_STATUS) :
      (data_enable_flag & DataBroadcast::DATA_ENABLE_FLAG::A3_HAS_STATUS);

  if ( flag_has_status)
  {
    Telemetry::TypeMap<Telemetry::TOPIC_STATUS_FLIGHT>::type fs =
        ptr_wrapper_->getStatus().flight;

    std_msgs::UInt8 flight_status;
    flight_status.data = fs;
    flight_status_publisher_.publish(flight_status);
  }

  uint16_t flag_has_gimbal =
      ptr_wrapper_->isM100() ? (data_enable_flag & DataBroadcast::DATA_ENABLE_FLAG::HAS_GIMBAL) :
      (data_enable_flag & DataBroadcast::DATA_ENABLE_FLAG::A3_HAS_GIMBAL);
  if (flag_has_gimbal)
  {
    Telemetry::Gimbal gimbal_reading;


    Telemetry::Gimbal gimbal_angle = ptr_wrapper_->getGimbal();

    geometry_msgs::Vector3Stamped gimbal_angle_vec3;

    gimbal_angle_vec3.header.stamp = now_time;
    gimbal_angle_vec3.header.frame_id = "ground_ENU";
    gimbal_angle_vec3.vector.x     = gimbal_angle.roll;
    gimbal_angle_vec3.vector.y     = gimbal_angle.pitch;
    gimbal_angle_vec3.vector.z     = gimbal_angle.yaw;
    gimbal_angle_publisher_.publish(gimbal_angle_vec3);
  }
}

double battery_perc=0.0;
double homepointHeight=0.0;

void VehicleNode::publish1HzData(Vehicle *vehicle, RecvContainer recvFrame, DJI::OSDK::UserData userData)
{
  VehicleNode *p = (VehicleNode *)userData;

  homepointHeight = vehicle->subscribe->getValue<Telemetry::TOPIC_ALTITUDE_OF_HOMEPOINT>();

  // Publish Camera List
    if (p->ptr_wrapper_->isM300()) {
        Telemetry::TypeMap<Telemetry::TOPIC_THREE_GIMBAL_DATA>::type gimbalData =
        vehicle->subscribe->getValue<Telemetry::TOPIC_THREE_GIMBAL_DATA>();

        //std::cout << "Got Gimbal Data\n";

        CameraManager* cameraMgr = p->ptr_wrapper_->getVehicle()->cameraManager;

        dji_sdk::ComponentList cameraList;
        std::time_t currentTime = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

        cameraList.timestamp = currentTime;
        cameraList.uid = p->boardIdSubstring;
        cameraList.seq = p->cameraListSeq;

        //vehicle->advancedSensing
        //p->ptr_wrapper_->getVehicle()->adva

        p->cameraListSeq ++;

        int cameraCount = 0;
        for (int i = 0; i < 3; i ++) {
            DJI::OSDK::Telemetry::GimbalSingleData gbComponentData = gimbalData.gbData[i];
            //std::cout << "gbComponentData:" << gbComponentData << "\n"

            //std::cout << "CameraName[" << i << "]: " << gbComponentData.mode << "\n";
            // std::cout << "Gimbal[" << i << "]: " << gbComponentData.status << "\n";
            // std::cout << "Mode[" << i << "]: " << gbComponentData.mode << "\n";

            //if (gbComponentData.status != -1) {
            
            std::string camPos;
            cameraMgr->getCameraModuleName(static_cast<PayloadIndexType>(i), camPos);
            std::string camVersion = cameraMgr->getCameraVersion(static_cast<PayloadIndexType>(i));
            std::string camFirmware = cameraMgr->getFirmwareVersion(static_cast<PayloadIndexType>(i));

            //std::cout << "CameraName[" << i << "]: " << camName << "\n";
            //std::cout << "CameraVersion[" << i << "]: " << camVersion << "\n";
            //std::cout << "CameraFirmware[" << i << "]: " << camFirmware << "\n";

            if (camVersion != "UNKNOWN" && camFirmware != "UNKNOWN") {
                cameraCount ++;
                cameraList.cameraVersion.push_back(camVersion + "@" + camFirmware);
                cameraList.cameraPosition.push_back(camPos);
                cameraList.payloadIndex.push_back(i);
            }
            //}
        }

        
        cameraList.cameraCount = cameraCount;
        //std::cout << "Camera Count:\t" << cameraList.cameraCount << "\n";

        p->camera_list_publisher_.publish(cameraList);

        // if (cameraCount > 0) {
        //     dji_sdk::Resolution streamRes;
        //     dji_sdk::Resolution photoRes;

        //     streamRes.height = (int)p->streamResolution.height;
        //     streamRes.width = (int)p->streamResolution.width;

        //     photoRes.height = (int)p->photoResolution.height;
        //     photoRes.width = (int)p->photoResolution.width;
        //     //cv::Size streamResolution = cv::Size(1280,720);
        //     //ros::Publisher main_camera_photo_publisher_;
        //     //cv::Size photoResolution = cv::Size(1280,720);


        //     p->main_camera_stream_resolution_publisher_.publish(streamRes);
        //     p->main_camera_photo_resolution_publisher_.publish(photoRes);

        //     // Camera Zoom factor publisher
        //     //std_msgs::Float32 cameraZoomFactorPckt;
        //     //float cameraZoomFactor = p->getZoomFactor(vehicle, PAYLOAD_INDEX_0, "H20T");
        //     //cameraZoomFactorPckt.data = (int)cameraZoomFactor;
        //     //p->main_camera_parameters_publisher_.publish(cameraZoomFactorPckt);
            
        //     //std::cout << "Polling Main Camera Contents\n";
        //     //p->requestStorageDeviceFiles(PAYLOAD_INDEX_0);
        // }
  }

  return;
}

float VehicleNode::getZoomFactor(Vehicle* vehicle, PayloadIndexType index, const char *name) {
    if (!vehicle || !vehicle->cameraManager) {
        DERROR("vehicle or cameraManager is a null value.");
        return ErrorCode::SysCommonErr::InstInitParamInvalid;
    }
    //ErrorCode::ErrorCodeType retCode;
    DJI::OSDK::CameraManager *pm = vehicle->cameraManager;


    char* msg;
    ErrorCode::ErrorCodeType ret; // = pm->initCameraModule(index, name);
    // if (ret != ErrorCode::SysCommonErr::Success)
    // {
    //     strcat(msg, "CameraCommands::getZoomFactor::Init Camera module ");
    //     strcat(msg, name);
    //     strcat(msg, " failed.\n");
    //     DERROR(msg);
    //     ErrorCode::printErrorCodeMsg(ret);
    //     return 0;
    // }

    float currFactor = 0.0;
    // auto lensInfo = cameraMgr->getLensInfo();
    // uint32_t curMs = 0;
    // OsdkOsal_GetTimeMs(&curMs);
    // /* Valid : data updated in 500 ms */
    // if ((lensInfo.updateTimeStamp <= curMs) && (lensInfo.updateTimeStamp >= curMs - 500)) {
    //   DSTATUS("Get lens data from pushing data.");
    //   if (cameraMgr)
    //   factor = 1.0f * lensInfo.data.current_focus_length
    //       / lensInfo.data.min_focus_length;
    //   DSTATUS("Getting zoom factor from lens info.");
    //   return ErrorCode::SysCommonErr::Success;
    // }
    ret = pm->getOpticalZoomFactorSync(index, currFactor, 1);
    if (ret != ErrorCode::SysCommonErr::Success) {
        strcat(msg, "CameraCommands::getZoomFactor::Zoom Camera ");
        strcat(msg, name);
        strcat(msg, " failed.\n");
        DERROR(msg);
        ErrorCode::printErrorCodeMsg(ret);
        return 0;
    }

    return currFactor;
}

void VehicleNode::requestStorageDeviceFiles(DJI::OSDK::PayloadIndexType PayloadIndex) {
    DJI::OSDK::CameraManager* cameraManager = ptr_wrapper_->getVehicle()->cameraManager;


    CameraModule::WorkMode camMode;
    cameraManager->getModeSync(PayloadIndex, camMode, 2);

    ErrorCode::ErrorCodeType ret;

    cameraManager->stopRecordVideoSync(PayloadIndex, 2);

    if (camMode != DJI::OSDK::CameraModule::WorkMode::PLAYBACK) {
        DSTATUS("Setting camera's work mode to PLAYBACK");
        ret = cameraManager->setModeSync(PayloadIndex,
                    DJI::OSDK::CameraModule::WorkMode::PLAYBACK, 2);

        if (ret != ErrorCode::SysCommonErr::Success)
        {
            DERROR(
                        "Could not set camera's work mode to playback using "
                        "'setModeSync(DJI::OSDK::PAYLOAD_INDEX_0, "
                        "DJI::OSDK::CameraModule::WorkMode::PLAYBACK, 2)'. "
                        "Error code: 0x%1X",
                        ret);
            ErrorCode::printErrorCodeMsg(ret);
        }
    } else {
        std::cout << "Camera Mode already set to PLAYBACK\n";
    }


    std::cout << "Camera Workmode:\t" << camMode << "\n";

    DSTATUS(
                "Obtain read permission to camera memory from camera, "
                "blocking calls");
    ret = cameraManager->obtainDownloadRightSync(
                PayloadIndex,
                true, 2);

    if (ret != ErrorCode::SysCommonErr::Success)
    {
        DERROR(
                    "Could not obtain read permission for camera storage. "
                    "Error code: 0x%1X",
                    ret);
        ErrorCode::printErrorCodeMsg(ret);
    }

    ret = cameraManager->startReqFileList(PayloadIndex, this->fileMgrFileListCb, this);

    sleep(5);  // wait 5 seconds for callback

    if (ret != ErrorCode::SysCommonErr::Success)
    {
        DERROR("Could not get file list. Error code: 0x%1X", ret);
        ErrorCode::printErrorCodeMsg(ret);
    }
}


void VehicleNode::fileMgrFileListCb(E_OsdkStat retCode, const FilePackage fileList, void* userData) {
    VehicleNode *p = (VehicleNode *)userData;

    if (retCode != E_OsdkStat::OSDK_STAT_OK)
    {
        DERROR(
            "Error receiving file list. Error code: "
            "0x%1X",
            retCode);
    }
    else
    {
        switch (fileList.type)
        {
        case DJI::OSDK::FileType::MEDIA:
            DSTATUS("Media file.");
            break;
        case DJI::OSDK::FileType::COMMON:
            DSTATUS("Common file.");
            break;
        case DJI::OSDK::FileType::SPEAKER_AUDIO:
            DSTATUS("Audio file.");
            break;
        default:
            DSTATUS("Unknown file type.");
            break;
        }


        std::cout << "\n";
        std::cout << "Number of files:\t" << fileList.media.size() << "\n";

        int64_t fileListSizeBytes = 0;
        float fileListSizeGb = 0.0;

        for (std::size_t i = 0; i < fileList.media.size(); ++i) {
            fileListSizeBytes += fileList.media[i].fileSize;
        }

        fileListSizeGb = fileListSizeBytes * 1e-9;
        std::cout << setprecision(2) << "H20T-Internal Storage:\t" << fileListSizeGb << "Gb\n";

        vector<dji_sdk::MediaFile> fileVector;

        std::cout << "valid\t" << "fileIndex\t" << "fileName\t" << "fileSize\t" << "duration\t" << "\n";
        for (int i = 0; i < fileList.media.size(); i++) {
            DJI::OSDK::MediaFile media = fileList.media[i];

            std::cout << std::fixed;
            std::cout << std::setprecision(2);
            std::cout << to_string(media.valid) << "\t" << to_string(media.fileIndex) << "\t" << media.fileName << "\t" << std::fixed << std::setprecision(2) << media.fileSize * 1e-6 << "mb\t" << to_string(media.duration) << "\n";

            dji_sdk::MediaFile mediaFile;

            mediaFile.fileIndex = media.fileIndex;
            mediaFile.fileName = media.fileName;
            mediaFile.fileSizeMb = media.fileSize * 1e-6;
            mediaFile.date = std::to_string(media.date.hour) + ":" + std::to_string(media.date.minute) + "-"
                    + std::to_string(media.date.day) + "/" + std::to_string(media.date.month) + "/" + std::to_string(media.date.year);

            fileVector.push_back(mediaFile);
        }

        dji_sdk::SDContent cameraContent;
        cameraContent.content = fileVector;

        std::cout << "Publishing Content List\n";
        p->main_camera_sd_contents_publisher_.publish(cameraContent);

        std::cout << "\n=========\nFile List END\n";
    }
}

void VehicleNode::publish5HzData(Vehicle *vehicle, RecvContainer recvFrame, DJI::OSDK::UserData userData)
{
  VehicleNode *p = (VehicleNode *)userData;

  uint8_t* data = recvFrame.recvData.raw_ack_array;
  ROS_ASSERT(static_cast<uint8_t>(SubscribePackgeIndex::PACKAGE_ID_5HZ) == *data );

  data++;
  Telemetry::TimeStamp packageTimeStamp = * (reinterpret_cast<Telemetry::TimeStamp *>(data));

  ros::Time msg_time = ros::Time::now();

  if(p->align_time_with_FC_)
  {
    if(p->curr_align_state_ == AlignStatus::ALIGNED)
    {
      msg_time = p->base_time_ + _TICK2ROSTIME(packageTimeStamp.time_ms);
    }
    else
    {
      return;
    }
  }

  //TODO: publish gps detail data if needed
  Telemetry::TypeMap<Telemetry::TOPIC_BATTERY_INFO>::type battery_info=
      vehicle->subscribe->getValue<Telemetry::TOPIC_BATTERY_INFO>();
  sensor_msgs::BatteryState msg_battery_state;
  msg_battery_state.header.stamp = msg_time;
  msg_battery_state.capacity = battery_info.capacity;
  msg_battery_state.voltage  = battery_info.voltage;
  msg_battery_state.current  = battery_info.current;
  msg_battery_state.percentage = battery_info.percentage;
  battery_perc = battery_info.percentage;
  msg_battery_state.charge   = NAN;
  msg_battery_state.design_capacity = NAN;
  msg_battery_state.power_supply_health = msg_battery_state.POWER_SUPPLY_HEALTH_UNKNOWN;
  msg_battery_state.power_supply_status = msg_battery_state.POWER_SUPPLY_STATUS_UNKNOWN;
  msg_battery_state.power_supply_technology = msg_battery_state.POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
  msg_battery_state.present = (battery_info.voltage!=0);
  p->battery_state_publisher_.publish(msg_battery_state);

  if(p->rtk_support_)
  {
    Telemetry::TypeMap<Telemetry::TOPIC_RTK_POSITION>::type rtk_telemetry_position=
        vehicle->subscribe->getValue<Telemetry::TOPIC_RTK_POSITION>();

    Telemetry::TypeMap<Telemetry::TOPIC_RTK_VELOCITY>::type rtk_telemetry_velocity=
        vehicle->subscribe->getValue<Telemetry::TOPIC_RTK_VELOCITY>();

    Telemetry::TypeMap<Telemetry::TOPIC_RTK_YAW>::type rtk_telemetry_yaw=
        vehicle->subscribe->getValue<Telemetry::TOPIC_RTK_YAW>();

    Telemetry::TypeMap<Telemetry::TOPIC_RTK_YAW_INFO>::type rtk_telemetry_yaw_info=
        vehicle->subscribe->getValue<Telemetry::TOPIC_RTK_YAW_INFO>();

    Telemetry::TypeMap<Telemetry::TOPIC_RTK_POSITION_INFO>::type rtk_telemetry_position_info=
        vehicle->subscribe->getValue<Telemetry::TOPIC_RTK_POSITION_INFO>();

    Telemetry::TypeMap<Telemetry::TOPIC_RTK_CONNECT_STATUS>::type rtk_telemetry_connect_status=
        vehicle->subscribe->getValue<Telemetry::TOPIC_RTK_CONNECT_STATUS>();

    sensor_msgs::NavSatFix rtk_position;
    rtk_position.header.stamp = msg_time;
    rtk_position.latitude = rtk_telemetry_position.latitude;
    rtk_position.longitude = rtk_telemetry_position.longitude;
    rtk_position.altitude = rtk_telemetry_position.HFSL;
    p->rtk_position_publisher_.publish(rtk_position);

    //! Velocity converted to m/s to conform to REP103.
    geometry_msgs::Vector3Stamped rtk_velocity;
    rtk_velocity.header.stamp = msg_time;
    rtk_velocity.vector.x = (rtk_telemetry_velocity.x)/100;
    rtk_velocity.vector.y = (rtk_telemetry_velocity.y)/100;
    rtk_velocity.vector.z = (rtk_telemetry_velocity.z)/100;
    p->rtk_velocity_publisher_.publish(rtk_velocity);

    std_msgs::Int16 rtk_yaw;
    rtk_yaw.data = rtk_telemetry_yaw;
    p->rtk_yaw_publisher_.publish(rtk_yaw);

    std_msgs::UInt8 rtk_yaw_info;
    rtk_yaw_info.data = (int)rtk_telemetry_yaw_info;
    p->rtk_yaw_info_publisher_.publish(rtk_yaw_info);

    std_msgs::UInt8 rtk_position_info;
    rtk_position_info.data = (int)rtk_telemetry_position_info;
    p->rtk_position_info_publisher_.publish(rtk_position_info);

    std_msgs::UInt8 rtk_connection_status;
    rtk_connection_status.data = (rtk_telemetry_connect_status.rtkConnected == 1) ? 1 : 0;
    p->rtk_connection_status_publisher_.publish(rtk_connection_status);
  }


  
  return;
}

tf::Quaternion q_FLU2ENU2;
tf::Quaternion quaternion2;
sensor_msgs::Joy rc_data_msg;

void VehicleNode::publish50HzData(Vehicle* vehicle, RecvContainer recvFrame,DJI::OSDK::UserData userData)
{
  VehicleNode* p = (VehicleNode*)userData;

  uint8_t* data = recvFrame.recvData.raw_ack_array;
  ROS_ASSERT(static_cast<uint8_t>(SubscribePackgeIndex::PACKAGE_ID_50HZ) == *data );
  data++;
  Telemetry::TimeStamp packageTimeStamp = * (reinterpret_cast<Telemetry::TimeStamp *>(data));

  ros::Time msg_time = ros::Time::now();

  if(p->align_time_with_FC_)
  {
    if(p->curr_align_state_ == AlignStatus::ALIGNED)
    {
      msg_time = p->base_time_ + _TICK2ROSTIME(packageTimeStamp.time_ms);
    }
    else
    {
      return;
    }
  }

  Telemetry::TypeMap<Telemetry::TOPIC_GPS_FUSED>::type fused_gps =
      vehicle->subscribe->getValue<Telemetry::TOPIC_GPS_FUSED>();
  Telemetry::TypeMap<Telemetry::TOPIC_ALTITUDE_FUSIONED>::type fused_altitude =
      vehicle->subscribe->getValue<Telemetry::TOPIC_ALTITUDE_FUSIONED>();


  sensor_msgs::NavSatFix gps_pos;
  gps_pos.header.frame_id = "/gps";
  gps_pos.header.stamp    = msg_time;
  gps_pos.latitude        = fused_gps.latitude * 180.0 / C_PI;   //degree
  gps_pos.longitude       = fused_gps.longitude * 180.0 / C_PI;  //degree
  gps_pos.altitude        = fused_altitude;                      //meter
  p->current_gps_latitude_ = gps_pos.latitude;
  p->current_gps_longitude_ = gps_pos.longitude;
  p->current_gps_altitude_ = fused_altitude;
  p->gps_position_publisher_.publish(gps_pos);

  if(p->local_pos_ref_set_)
  {
    geometry_msgs::PointStamped local_pos;
    local_pos.header.frame_id = "/local";
    local_pos.header.stamp = gps_pos.header.stamp;
    p->gpsConvertENU(local_pos.point.x, local_pos.point.y, gps_pos.longitude,
                     gps_pos.latitude, p->local_pos_ref_longitude_, p->local_pos_ref_latitude_);
    local_pos.point.z = gps_pos.altitude - p->local_pos_ref_altitude_;
    /*!
    * note: We are now following REP 103 to use ENU for
    *       short-range Cartesian representations. Local position is published
    *       in ENU Frame
    */
    p->local_position_publisher_.publish(local_pos);
  }

  Telemetry::TypeMap<Telemetry::TOPIC_HEIGHT_FUSION>::type fused_height =
      vehicle->subscribe->getValue<Telemetry::TOPIC_HEIGHT_FUSION>();
  std_msgs::Float32 height;
  height.data = fused_height;
  p->height_publisher_.publish(height);

  Telemetry::TypeMap<Telemetry::TOPIC_STATUS_FLIGHT>::type fs =
      vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_FLIGHT>();

  std_msgs::UInt8 flight_status;
  flight_status.data = fs;
  p->flight_status_publisher_.publish(flight_status);

  Telemetry::TypeMap<Telemetry::TOPIC_VELOCITY>::type v_FC =
      vehicle->subscribe->getValue<Telemetry::TOPIC_VELOCITY>();
  geometry_msgs::Vector3Stamped v;
  // v_FC has 2 fields, data and info. The latter contains the health

  //std::cout << "Vel x:" << v_FC.data.y << "\tVel y:" << v_FC.data.x << "\tVel z:" << v_FC.data.z << "\n";

  /*!
   * note: We are now following REP 103 to use ENU for
   *       short-range Cartesian representations
   */
  v.header.frame_id = "ground_ENU";
  v.header.stamp = msg_time;
  v.vector.x = v_FC.data.y;  //x, y are swapped from NE to EN
  v.vector.y = v_FC.data.x;
  v.vector.z = v_FC.data.z; //z sign is already U
  p->velocity_publisher_.publish(v);

  Telemetry::TypeMap<Telemetry::TOPIC_GPS_CONTROL_LEVEL>::type gps_ctrl_level=
      vehicle->subscribe->getValue<Telemetry::TOPIC_GPS_CONTROL_LEVEL>();
  std_msgs::UInt8 msg_gps_ctrl_level;
  msg_gps_ctrl_level.data = gps_ctrl_level;
  p->current_gps_health_ = gps_ctrl_level;
  p->gps_health_publisher_.publish(msg_gps_ctrl_level);

  Telemetry::TypeMap<Telemetry::TOPIC_GIMBAL_ANGLES>::type gimbal_angle =
      vehicle->subscribe->getValue<Telemetry::TOPIC_GIMBAL_ANGLES>();

  geometry_msgs::Vector3Stamped gimbal_angle_vec3;

  gimbal_angle_vec3.header.stamp = ros::Time::now();
  gimbal_angle_vec3.header.frame_id = "ground_ENU";
  gimbal_angle_vec3.vector.x     = gimbal_angle.x;
  gimbal_angle_vec3.vector.y     = gimbal_angle.y;
  gimbal_angle_vec3.vector.z     = gimbal_angle.z;
  p->gimbal_angle_publisher_.publish(gimbal_angle_vec3);

  // See dji_sdk.h for details about display_mode

  Telemetry::TypeMap<Telemetry::TOPIC_STATUS_DISPLAYMODE>::type dm =
      vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>();

  std_msgs::UInt8 status_dm;
  status_dm.data = dm;
  p->displaymode_publisher_.publish(status_dm);

  /*!
   * note: Since FW version 3.3.0 and SDK version 3.7, we expose all the button on the LB2 RC
   *       as well as the RC connection status via different topics.
   */
  if(vehicle->getFwVersion() > versionBase33)
  {
    Telemetry::TypeMap<Telemetry::TOPIC_POSITION_VO>::type vo_position =
        vehicle->subscribe->getValue<Telemetry::TOPIC_POSITION_VO>();

    dji_sdk::VOPosition vo_pos;
    // This name does not follow the convention because we are not sure it is real NED.
    vo_pos.header.frame_id = "/ground_nav";
    vo_pos.header.stamp = msg_time;
    vo_pos.x  = vo_position.x;
    vo_pos.y  = vo_position.y;
    vo_pos.z  = vo_position.z;
    vo_pos.xHealth = vo_position.xHealth;
    vo_pos.yHealth = vo_position.yHealth;
    vo_pos.zHealth = vo_position.zHealth;
    p->vo_position_publisher_.publish(vo_pos);

    Telemetry::TypeMap<Telemetry::TOPIC_RC_WITH_FLAG_DATA>::type rc_with_flag =
        vehicle->subscribe->getValue<Telemetry::TOPIC_RC_WITH_FLAG_DATA>();

//    Telemetry::TypeMap<Telemetry::TOPIC_RC_FULL_RAW_DATA>::type rc_full_raw =
//        vehicle->subscribe->getValue<Telemetry::TOPIC_RC_FULL_RAW_DATA>();

    //std::cout << "rc full raw:\t" << std::to_string(rc_full_raw) << "\n";

    sensor_msgs::Joy rc_joy;
    rc_joy.header.stamp    = msg_time;
    rc_joy.header.frame_id = "rc33";

    rc_joy.axes.reserve(12);
//        rc_joy.axes.push_back(static_cast<float>(rc_full_raw.lb2.roll));
//        rc_joy.axes.push_back(static_cast<float>(rc_full_raw.lb2.pitch));
//        rc_joy.axes.push_back(static_cast<float>(rc_full_raw.lb2.yaw));
//        rc_joy.axes.push_back(static_cast<float>(rc_full_raw.lb2.throttle));

    rc_joy.axes.push_back(static_cast<float>(rc_with_flag.roll));
    rc_joy.axes.push_back(static_cast<float>(rc_with_flag.pitch));
    rc_joy.axes.push_back(static_cast<float>(rc_with_flag.yaw));
    rc_joy.axes.push_back(static_cast<float>(rc_with_flag.throttle));
    if (static_cast<float>(rc_with_flag.roll) >= 0.98)
	if(static_cast<float>(rc_with_flag.yaw) >= 0.98)
		startTrackingFlag = true;
    // A3 and N3 has access to more buttons on RC
    std::string hardwareVersion(vehicle->getHwVersion());
    //std::cout << "Hardware version:\t" << hardwareVersion << "\n";
    if( (hardwareVersion == std::string(Version::N3)) || hardwareVersion == std::string(Version::A3))
    {
        std::cout << "Advanced RC enabled\n";
      Telemetry::TypeMap<Telemetry::TOPIC_RC_FULL_RAW_DATA>::type rc_full_raw =
          vehicle->subscribe->getValue<Telemetry::TOPIC_RC_FULL_RAW_DATA>();
      rc_joy.axes.push_back(static_cast<float>(-(rc_full_raw.lb2.mode - 1024)    / 660));
      rc_joy.axes.push_back(static_cast<float>(-(rc_full_raw.lb2.gear - 1519)    / 165));
      rc_joy.axes.push_back(static_cast<float>((rc_full_raw.lb2.camera -364)   / 1320));
      rc_joy.axes.push_back(static_cast<float>((rc_full_raw.lb2.video - 364)    / 1320));
      rc_joy.axes.push_back(static_cast<float>((rc_full_raw.lb2.videoPause-364) / 1320));
      rc_joy.axes.push_back(static_cast<float>((rc_full_raw.lb2.goHome-364) /1320));
      rc_joy.axes.push_back(static_cast<float>((rc_full_raw.lb2.leftWheel-1024.0)  / 660.0));
      rc_joy.axes.push_back(static_cast<float>((rc_full_raw.lb2.rightWheelButton - 364)/ 1320));

      rc_joy.buttons.reserve(2);
      rc_joy.buttons.push_back(static_cast<float>(rc_full_raw.lb2.rcC1));
      rc_joy.buttons.push_back(static_cast<float>(rc_full_raw.lb2.rcC2));
    }
    else
    {
        //std::cout << "Weird RC enabled\n";
      Telemetry::TypeMap<Telemetry::TOPIC_RC>::type rc =
          vehicle->subscribe->getValue<Telemetry::TOPIC_RC>();

//      Telemetry::TypeMap<Telemetry::TOPIC_RC_FULL_RAW_DATA>::type rc_full_raw =
//          vehicle->subscribe->getValue<Telemetry::TOPIC_RC_FULL_RAW_DATA>();

      rc_joy.axes.push_back(static_cast<float>(rc.mode*1.0));
      rc_joy.axes.push_back(static_cast<float>(rc.gear*1.0));
      //rc_joy.axes.push_back(static_cast<float>(rc_full_raw.lb2.mode*1.0));
      //rc_joy.axes.push_back(static_cast<float>(rc_full_raw.lb2.gear*1.0));

//      rc_joy.buttons.reserve(10);
//      int16_t rcC1;             /*!< press_down = 1684, release = 364 */
//      int16_t rcC2;             /*!< press_down = 1684, release = 364 */
//      int16_t rcD1;             /*!< rcD1 - 8 is used by sbus */
//      int16_t rcD2;
//      int16_t rcD3;
//      int16_t rcD4;
//      int16_t rcD5;
//      int16_t rcD6;
//      int16_t rcD7;
//      int16_t rcD8;
//      rc_joy.buttons.push_back(static_cast<float>(rc_full_raw.lb2.rcC1));
//      rc_joy.buttons.push_back(static_cast<float>(rc_full_raw.lb2.rcC2));

//      rc_joy.buttons.push_back(static_cast<float>(rc_full_raw.lb2.rcD1));
//      rc_joy.buttons.push_back(static_cast<float>(rc_full_raw.lb2.rcD2));

//      rc_joy.buttons.push_back(static_cast<float>(rc_full_raw.lb2.rcD3));
//      rc_joy.buttons.push_back(static_cast<float>(rc_full_raw.lb2.rcD4));

//      rc_joy.buttons.push_back(static_cast<float>(rc_full_raw.lb2.rcD5));
//      rc_joy.buttons.push_back(static_cast<float>(rc_full_raw.lb2.rcD6));

//      rc_joy.buttons.push_back(static_cast<float>(rc_full_raw.lb2.rcD7));
//      rc_joy.buttons.push_back(static_cast<float>(rc_full_raw.lb2.rcD8));

//      std::cout << "rcC1:\t" << std::to_string(rc_full_raw.lb2.rcC1) << "\n";
//      std::cout << "rcC2:\t" << std::to_string(rc_full_raw.lb2.rcC2) << "\n";

      if (static_cast<float>(rc.mode*1.0) == 8000){
	pMode = true;
      }else{
	pMode = false;
	sModeTriggered = true;
      }
    }
    
    rc_data_msg = rc_joy;
    p->rc_publisher_.publish(rc_joy);

    bool temp;
    temp = rc_with_flag.flag.skyConnected && rc_with_flag.flag.groundConnected;

    std_msgs::UInt8 rc_connected;
    rc_connected.data = temp ? 1 : 0;
    p->rc_connection_status_publisher_.publish(rc_connected);

    // Publish flight anomaly if FC is supported
    Telemetry::TypeMap<Telemetry::TOPIC_FLIGHT_ANOMALY>::type flight_anomaly_data =
        vehicle->subscribe->getValue<Telemetry::TOPIC_FLIGHT_ANOMALY>();

    dji_sdk::FlightAnomaly flight_anomaly_msg;
    flight_anomaly_msg.data = *(reinterpret_cast<uint32_t*>(&flight_anomaly_data));
    p->flight_anomaly_publisher_.publish(flight_anomaly_msg);
  }
  else
  {
    /********* RC Map (A3) *********
    *
    *       -10000  <--->  0      <---> 10000
    * MODE: API(F)  <---> ATTI(A) <--->  POS (P)
    *
    *        CH3 +10000                     CH1 +10000
    *               ^                              ^
    *               |                              |                   / -5000
    *    CH2        |                   CH0        |                  /
    *  -10000 <-----------> +10000    -10000 <-----------> +10000    H
    *               |                              |                  \
    *               |                              |                   \ -10000
    *               V                              V
    *            -10000                         -10000
    *
    *   In this code, before publishing, we normalize RC
    *****************************/

    Telemetry::TypeMap<Telemetry::TOPIC_RC>::type rc =
        vehicle->subscribe->getValue<Telemetry::TOPIC_RC>();

    sensor_msgs::Joy rc_joy;
    rc_joy.header.stamp    = msg_time;
    rc_joy.header.frame_id = "rcElse";

    rc_joy.axes.reserve(6);

    rc_joy.axes.push_back(static_cast<float>(rc.roll     / 10000.0));
    rc_joy.axes.push_back(static_cast<float>(rc.pitch    / 10000.0));
    rc_joy.axes.push_back(static_cast<float>(rc.yaw      / 10000.0));
    rc_joy.axes.push_back(static_cast<float>(rc.throttle / 10000.0));
    rc_joy.axes.push_back(static_cast<float>(rc.mode*1.0));
    rc_joy.axes.push_back(static_cast<float>(rc.gear*1.0));
    p->rc_publisher_.publish(rc_joy);
  }

  Telemetry::TypeMap<Telemetry::TOPIC_ESC_DATA>::type esc_data =
          vehicle->subscribe->getValue<Telemetry::TOPIC_ESC_DATA>();

  // Create EscData message
  dji_sdk::EscData escMsg;


  // Create Vector
  vector<dji_sdk::ESCStatusIndividual> motorVector;

  for (auto esc : esc_data.esc) {
      dji_sdk::ESCStatusIndividual escIndividual;

      escIndividual.current = esc.current;
      escIndividual.speed = esc.speed;
      escIndividual.voltage = esc.voltage;
      escIndividual.temperature = esc.temperature;
      escIndividual.stall = esc.stall;
      escIndividual.empty = esc.empty;
      escIndividual.unbalanced = esc.unbalanced;
      escIndividual.escDisconnected = esc.escDisconnected;
      escIndividual.temperatureHigh = esc.temperatureHigh;
      escIndividual.reserved = esc.reserved;

      motorVector.push_back(escIndividual);
  }

  escMsg.esc.resize(motorVector.size());

  dji_sdk::ESCStatusIndividual escArr[motorVector.size()];
  std::copy(motorVector.begin(), motorVector.end(), escArr);

  escMsg.esc = motorVector;

  // Publish EscData message
  p->esc_data_publisher_.publish(escMsg);


  int16_t esc_speed[4];
  for (int escNum=0 ; escNum < 4 ; escNum++)
  {
      esc_speed[escNum] = esc_data.esc[escNum].speed;
  }

  // Telemetry Translation

  dji_sdk::WindData windData;
  dji_sdk::telemetry2 telemetry;

  telemetry.rostime_secs = msg_time.sec;
  telemetry.rostime_nsecs = msg_time.nsec;
  telemetry.batteryPercentage = battery_perc;

  telemetry.altitude = fused_altitude - homepointHeight;  //meter
  telemetry.longitude = fused_gps.longitude * 180.0 / C_PI;  //degree
  telemetry.latitude = fused_gps.latitude * 180.0 / C_PI;   //degree
  telemetry.gpsSignal = gps_ctrl_level;
  telemetry.satelliteNumber = fused_gps.visibleSatelliteNumber;


  //double siny_cosp = 2.0 * (q_FLU2ENU2.getW() * q_FLU2ENU2.getZ() + q_FLU2ENU2.getX() * q_FLU2ENU2.getY());
  //double cosy_cosp = 1.0 - 2.0 * (q_FLU2ENU2.getY() * q_FLU2ENU2.getY() + q_FLU2ENU2.getZ() * q_FLU2ENU2.getZ());
  double siny_cosp = 2.0 * (quaternion2.getW() * quaternion2.getZ() + quaternion2.getX() * quaternion2.getY());
  double cosy_cosp = 1.0 - 2.0 * (quaternion2.getY() * quaternion2.getY() + quaternion2.getZ() * quaternion2.getZ());
  double yaw = atan2(cosy_cosp, siny_cosp) * 180 / C_PI;

  //telemetry.heading =  ((yaw * -100.0f) / 1.75f)+90.0f;
  telemetry.heading =  yaw;

  double one = 2.0 * (q_FLU2ENU2.getW() * q_FLU2ENU2.getX() + q_FLU2ENU2.getY() * q_FLU2ENU2.getZ());
  double two = 1.0 - 2.0 * (q_FLU2ENU2.getX() * q_FLU2ENU2.getX() + q_FLU2ENU2.getY() * q_FLU2ENU2.getY());
  double fi = atan2(one, two);

  double theta = asin(2.0 * (q_FLU2ENU2.getW() * q_FLU2ENU2.getY() - q_FLU2ENU2.getZ() * q_FLU2ENU2.getX()));

  double alpha = acos(cos(theta) * cos(fi));
  double coeff;

  double windDirection;

  double three = (-1.0 * sin(fi) * cos(theta));
  double four = (cos(fi) * sin(theta));
  windDirection = atan(three/four) * 180.0 / C_PI;

  std::string line;
  double windSpeed;
  //std::cout << "	theta: " << theta << "	fi: " << fi << std::endl;

  std::ifstream file("/home/jetson/catkin_ws/wind.txt");
  if (file.is_open()){
      getline(file, line);
      std::stringstream stream(line);
      stream >> coeff;
      windSpeed = sqrt(coeff * tan(alpha));
      if (fi >= 0 && theta >=0){
          windDirection = windDirection;
      }else if (fi >= 0 && theta < 0){
          windDirection = windDirection + 180.0;
      }else if (fi < 0 && theta >= 0){
          windDirection = windDirection;
      }else if (fi < 0 && theta < 0){
          windDirection = windDirection + 180.0;
      }
      windDirection = yaw - windDirection;
      if (windDirection < -180.0){
          windDirection = 360.0 + windDirection;
      }
  }
  file.close();


  double x    =  v_FC.data.x;  //x, y are swapped from NE to ENs
  double y    =  v_FC.data.y;
  //double z    = -vehicle->broadcast->getAngularRate().z;
  double a = sqrt(x*x + y*y);
  double magnitude = sqrt(x * x + y * y);
  //std::cout << "x:" << x << "\ty:" << y << "\tmagnitude:" << magnitude << "\n";
  //double b = sqrt(a*a + z*z);

  

  telemetry.velocity = magnitude;
  

  windData.windSpeed = windSpeed;
  windData.windDirection = windDirection;

  p->telemetry_tanslator_publisher_.publish(telemetry);
  p->wind_data_publisher_.publish(windData);
}

void VehicleNode::publish100HzData(Vehicle *vehicle, RecvContainer recvFrame, DJI::OSDK::UserData userData)
{
  VehicleNode *p = (VehicleNode *)userData;

  uint8_t* data = recvFrame.recvData.raw_ack_array;
  ROS_ASSERT(static_cast<uint8_t>(SubscribePackgeIndex::PACKAGE_ID_100HZ) == *data );
  data++;
  Telemetry::TimeStamp packageTimeStamp = * (reinterpret_cast<Telemetry::TimeStamp *>(data));

  ros::Time msg_time = ros::Time::now();

  if(p->align_time_with_FC_)
  {
    if(p->curr_align_state_ == AlignStatus::ALIGNED)
    {
      msg_time = p->base_time_ + _TICK2ROSTIME(packageTimeStamp.time_ms);
    }
    else
    {
      return;
    }
  }

  Telemetry::TypeMap<Telemetry::TOPIC_QUATERNION>::type quat =
      vehicle->subscribe->getValue<Telemetry::TOPIC_QUATERNION>();
  geometry_msgs::QuaternionStamped q;

  /*!
   * note: We are now following REP 103 to use FLU for
   *       body frame. The quaternion is the rotation from
   *       body_FLU to ground_ENU
   */
  q.header.frame_id = "body_FLU";
  q.header.stamp    = msg_time;

  tf::Matrix3x3 R_FRD2NED(tf::Quaternion(quat.q1, quat.q2, quat.q3, quat.q0));
  tf::Matrix3x3 R_FLU2ENU = p->R_ENU2NED_.transpose() * R_FRD2NED * p->R_FLU2FRD_;
  tf::Quaternion q_FLU2ENU;

  R_FLU2ENU.getRotation(q_FLU2ENU);
  R_FLU2ENU.getRotation(q_FLU2ENU2);
  R_FLU2ENU.getRotation(quaternion2);
  // @note this mapping is tested
  q.quaternion.w = q_FLU2ENU.getW();
  q.quaternion.x = q_FLU2ENU.getX();
  q.quaternion.y = q_FLU2ENU.getY();
  q.quaternion.z = q_FLU2ENU.getZ();
  p->attitude_publisher_.publish(q);

  Telemetry::TypeMap<Telemetry::TOPIC_ANGULAR_RATE_FUSIONED>::type w_FC =
      vehicle->subscribe->getValue<Telemetry::TOPIC_ANGULAR_RATE_FUSIONED>();

  geometry_msgs::Vector3Stamped angular_rate;

  /*!
   * note: We are now following REP 103 to use FLU for
   *       body frame
   */
  angular_rate.header.frame_id = "body_FLU";
  angular_rate.header.stamp    = msg_time;

  angular_rate.vector.x        =  w_FC.x;
  angular_rate.vector.y        = -w_FC.y; //y,z sign are flipped from RD to LU
  angular_rate.vector.z        = -w_FC.z;
  p->angularRate_publisher_.publish(angular_rate);

  Telemetry::TypeMap<Telemetry::TOPIC_ACCELERATION_GROUND>::type a_FC =
      vehicle->subscribe->getValue<Telemetry::TOPIC_ACCELERATION_GROUND>();
  geometry_msgs::Vector3Stamped acceleration;

  /*!
   * note: 1. We are now following REP 103 to use ENU for
   *       short-range Cartesian representations
   *
   *       2. TODO: This accel is in ground frame, which may
   *       cause confusion with the body-frame accel in imu message
   */

  acceleration.header.frame_id = "ground_ENU";
  acceleration.header.stamp    = msg_time;

  acceleration.vector.x        = a_FC.y;  //x, y are swapped from NE to EN
  acceleration.vector.y        = a_FC.x;
  acceleration.vector.z        = a_FC.z;  //z sign is already U
  p->acceleration_publisher_.publish(acceleration);
}

void VehicleNode::publish400HzData(Vehicle *vehicle, RecvContainer recvFrame, DJI::OSDK::UserData userData)
{
  VehicleNode *p = (VehicleNode *) userData;

  uint8_t* data = recvFrame.recvData.raw_ack_array;
  ROS_ASSERT(static_cast<uint8_t>(SubscribePackgeIndex::PACKAGE_ID_400HZ) == *data );

  data++;
  Telemetry::TimeStamp packageTimeStamp = * (reinterpret_cast<Telemetry::TimeStamp *>(data));

  Telemetry::TypeMap<Telemetry::TOPIC_HARD_SYNC>::type hardSync_FC =
      vehicle->subscribe->getValue<Telemetry::TOPIC_HARD_SYNC>();

  ros::Time now_time = ros::Time::now();
  ros::Time msg_time = now_time;

  if(p->align_time_with_FC_)
  {
    p->alignRosTimeWithFlightController(now_time, packageTimeStamp.time_ms);
    if(p->curr_align_state_ == AlignStatus::ALIGNED)
    {
      msg_time = p->base_time_ + _TICK2ROSTIME(packageTimeStamp.time_ms);
    }
    else
    {
      return;
    }
  }

  sensor_msgs::Imu synced_imu;

  synced_imu.header.frame_id = "body_FLU";
  synced_imu.header.stamp    = msg_time;

  //y, z signs are flipped from RD to LU for rate and accel
  synced_imu.angular_velocity.x    =   hardSync_FC.w.x;
  synced_imu.angular_velocity.y    =  -hardSync_FC.w.y;
  synced_imu.angular_velocity.z    =  -hardSync_FC.w.z;

  synced_imu.linear_acceleration.x =   hardSync_FC.a.x * p->gravity_const_;
  synced_imu.linear_acceleration.y =  -hardSync_FC.a.y * p->gravity_const_;
  synced_imu.linear_acceleration.z =  -hardSync_FC.a.z * p->gravity_const_;

  /*!
   * The quaternion is the rotation from body_FLU to ground_ENU.
   * Refer to:
   *   https://github.com/mavlink/mavros/blob/master/mavros/src/plugins/imu_pub.cpp
   */
  tf::Matrix3x3 R_FRD2NED(tf::Quaternion(hardSync_FC.q.q1, hardSync_FC.q.q2,
                                         hardSync_FC.q.q3, hardSync_FC.q.q0));
  tf::Matrix3x3 R_FLU2ENU = p->R_ENU2NED_.transpose() * R_FRD2NED * p->R_FLU2FRD_;
  tf::Quaternion q_FLU2ENU;
  R_FLU2ENU.getRotation(q_FLU2ENU);

  synced_imu.orientation.w = q_FLU2ENU.getW();
  synced_imu.orientation.x = q_FLU2ENU.getX();
  synced_imu.orientation.y = q_FLU2ENU.getY();
  synced_imu.orientation.z = q_FLU2ENU.getZ();

  p->imu_publisher_.publish(synced_imu);

  if (hardSync_FC.ts.flag == 1)
  {
    sensor_msgs::TimeReference trigTime;
    trigTime.header.stamp = msg_time;
    trigTime.time_ref     = now_time;
    trigTime.source       = "FC";

    p->trigger_publisher_.publish(trigTime);
  }
}

/*!
 * @brief: The purpose of time alignment is to use the time received from flight
 *         controller to stamp all published ros messages. The reason is that the
 *         flight controller is running a real time system, while the ros time can
 *         be affected by OS scheduling depending on system load.
 */

void VehicleNode::alignRosTimeWithFlightController(ros::Time now_time, uint32_t tick)
{
  if (curr_align_state_ == AlignStatus::UNALIGNED)
  {
    base_time_ = now_time - _TICK2ROSTIME(tick);
    curr_align_state_ = AlignStatus::ALIGNING;
    ROS_INFO("[dji_sdk] Start time alignment ...");
    return;
  }

  if (curr_align_state_ == AlignStatus::ALIGNING)
  {
    static int aligned_count = 0;
    static int retry_count = 0;
    ROS_INFO_THROTTLE(1.0, "[dji_sdk] Aliging time...");

    double dt = std::fabs((now_time - (base_time_ + _TICK2ROSTIME(tick))).toSec());

    if(dt < TIME_DIFF_CHECK )
    {
      aligned_count++;
    }
    else if(aligned_count > 0)
    {
      base_time_ = now_time - _TICK2ROSTIME(tick);
      ROS_INFO("[dji_sdk] ***** Time difference out of bound after %d samples, retried %d times, dt=%.3f... *****",
               aligned_count, retry_count, dt);
      aligned_count = 0;
      retry_count++;
    }

    if(aligned_count > STABLE_ALIGNMENT_COUNT)
    {
      ROS_INFO("[dji_sdk] ***** Time alignment successful! *****");
      curr_align_state_ = AlignStatus::ALIGNED;
    }

    return;
  }
}

void VehicleNode::gpsConvertENU(double &ENU_x, double &ENU_y, double gps_t_lon, double gps_t_lat, double gps_r_lon, double gps_r_lat)
{
  double d_lon = gps_t_lon - gps_r_lon;
  double d_lat = gps_t_lat - gps_r_lat;
  ENU_y = DEG2RAD(d_lat) * C_EARTH;
  ENU_x = DEG2RAD(d_lon) * C_EARTH * cos(DEG2RAD(gps_t_lat));
}

#ifdef ADVANCED_SENSING
void VehicleNode::publish240pStereoImage(Vehicle* vehicle,RecvContainer recvFrame,DJI::OSDK::UserData userData)
{
  VehicleNode *node_ptr = (VehicleNode *)userData;

  node_ptr->stereo_subscription_success = true;

  sensor_msgs::Image img;
  img.height = 240;
  img.width = 320;
  img.data.resize(img.height*img.width);
  img.encoding = "mono8";
  img.step = 320;
  uint8_t img_idx = 0;

  for (int pair_idx = 0; pair_idx < DJI::OSDK::CAMERA_PAIR_NUM; ++pair_idx) {
    for (int dir_idx = 0; dir_idx < DJI::OSDK::IMAGE_TYPE_NUM; ++dir_idx) {

      uint8_t bit_location = pair_idx * IMAGE_TYPE_NUM + dir_idx;
      uint8_t bit_val = (recvFrame.recvData.stereoImgData->img_desc >> bit_location) & 1;

      if (bit_val) {
        img.header.seq = recvFrame.recvData.stereoImgData->frame_index;
        img.header.stamp = ros::Time::now(); // @todo
        img.header.frame_id = recvFrame.recvData.stereoImgData->img_vec[img_idx].name;
        memcpy((char*)(&img.data[0]), recvFrame.recvData.stereoImgData->img_vec[img_idx++].image, 240*320);

        if (bit_location == static_cast<uint8_t>(dji_sdk::ReceivedImgDesc::RECV_FRONT_LEFT))
          node_ptr->stereo_240p_front_left_publisher_.publish(img);
        if (bit_location == static_cast<uint8_t>(dji_sdk::ReceivedImgDesc::RECV_FRONT_RIGHT))
          node_ptr->stereo_240p_front_right_publisher_.publish(img);
        if (bit_location == static_cast<uint8_t>(dji_sdk::ReceivedImgDesc::RECV_DOWN_BACK))
          node_ptr->stereo_240p_down_back_publisher_.publish(img);
        if (bit_location == static_cast<uint8_t>(dji_sdk::ReceivedImgDesc::RECV_DOWN_FRONT))
          node_ptr->stereo_240p_down_front_publisher_.publish(img);
        if (bit_location == static_cast<uint8_t>(dji_sdk::ReceivedImgDesc::RECV_FRONT_DEPTH))
          node_ptr->stereo_240p_front_depth_publisher_.publish(img);
      }
    }
  }
}


void VehicleNode::PerceptionImageCB(Perception::ImageInfoType info, uint8_t *imageRawBuffer,
                       int bufferLen, void *userData) {
    DSTATUS("image info: dataId(%d) seq(%d) timestamp(%llu) datatype(%d) index(%d) h(%d) w(%d) dir(%d) "
            "bpp(%d) bufferlen(%d)", info.dataId, info.sequence, info.timeStamp, info.dataType,
            info.rawInfo.index, info.rawInfo.height, info.rawInfo.width, info.rawInfo.direction,
            info.rawInfo.bpp, bufferLen);
    
    // Add camera Identifier

    StereoImagePacketType *pack;
    VehicleNode *node_ptr;

    if (userData) {
        VehicleStereoImagePacketType* vehImageType = (VehicleStereoImagePacketType*)userData;
        
        pack = vehImageType->stereoImagePacketType;
        node_ptr = vehImageType->vh_node;
        std::cout << "Got VehicleNode and pack\n";
    }

    if (imageRawBuffer && userData) {
        std::cout << "imageRawBuffer or userData exist\n";
        std::cout << "setting StereoImagePacketType *pack\n";
        std::cout << "calling OsdkOsal_MutexLock\n";

        OsdkOsal_MutexLock(pack->mutex);
            std::cout << "setting pack->info\n";
            pack->info = info;
            
            if (pack->imageRawBuffer) {
                std::cout << "calling OsdkOsal_Free\n";
                OsdkOsal_Free(pack->imageRawBuffer);
            }

            std::cout << "setting imageRawBuffer\n";
            pack->imageRawBuffer = (uint8_t *)OsdkOsal_Malloc(bufferLen);
            //memcpy(pack->imageRawBuffer, imageRawBuffer, bufferLen);

            std::cout << "creating image packet\n";
            sensor_msgs::Image img;
            img.height = info.rawInfo.height;
            img.width = info.rawInfo.width;
            img.data.resize(img.height*img.width);
            img.encoding = "mono8";
            img.step = img.width;
            uint8_t img_idx = 0;

            if (info.dataType == 1) {
                img.header.frame_id = "Left"; //recvFrame.recvData.stereoImgData->img_vec[img_idx].name;
            } else {
                img.header.frame_id = "Right";
            }

            img.header.seq = info.sequence;//recvFrame.recvData.stereoImgData->frame_index;
            img.header.stamp = ros::Time::now(); // @todo
            
            std::cout << "copying to packet\n";
            std::cout << "Img Size:" << std::to_string(info.rawInfo.width) << "x" << std::to_string(info.rawInfo.height) << "\n";
            std::cout << "bufferLen:" << std::to_string(bufferLen) << "\n";
            memcpy(&img.data[0], imageRawBuffer, bufferLen);
            //memcpy((char*)(&img.data[0]), imageRawBuffer, bufferLen);

            std::cout << "publishing....\n";
            node_ptr->stereo_depth_publisher_.publish(img);

            std::cout << "setting pack->gotData\n";
            pack->gotData = true;
            std::cout << "calling OsdkOsal_MutexUnlock\n";
        OsdkOsal_MutexUnlock(pack->mutex);
        std::cout << "calling OsdkOsal_MutexUnlock DONE\n";



    } else {
        std::cout << "imageRawBuffer or userData missing\n";
    }
}

// Draw the predicted bounding box
void drawPred(int classId, float conf, int left, int top, int right, int bottom, Mat& frame, int mode)
{
    //Draw a rectangle displaying the bounding box
    rectangle(frame, Point(left, top), Point(right, bottom), Scalar(255, 178, 50), 3);

    //Get the label for the class name and its confidence
    string label = format("%.2f", conf);
    vector<string> classes;
    if (mode == 0)
        classes = VehicleNode::getClasses();
    else
        classes = VehicleNode::getClassesHarpy();
    if (!classes.empty())
    {
        CV_Assert(classId < (int)classes.size());
        label = classes[classId] + ":" + label;
    }

    //Display the label at the top of the bounding box
    int baseLine;
    Size labelSize = getTextSize(label, FONT_HERSHEY_TRIPLEX, 0.5, 1, &baseLine);
    top = max(top, labelSize.height);
    rectangle(frame, Point(left, top - round(1.5*labelSize.height)), Point(left + round(1.5*labelSize.width), top + baseLine), Scalar(255, 255, 255), FILLED);
    putText(frame, label, Point(left, top), FONT_HERSHEY_TRIPLEX, 0.75, Scalar(0,0,0),1);
}

// Remove the bounding boxes with low confidence using non-maxima suppression
void postprocess1(Mat& frame, const vector<Mat>& outs, int mode)
{
    classIds.clear();
    confidences.clear();
    boxes2.clear();

    for (size_t i = 0; i < outs.size(); ++i)
    {
        // Scan through all the bounding boxes output from the network and keep only the
        // ones with high confidence scores. Assign the box's class label as the class
        // with the highest score for the box.
        float* data = (float*)outs[i].data;
        for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
        {
            Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
            Point classIdPoint;
            double confidence;
            // Get the value and location of the maximum score
            minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
            if (confidence > confThreshold)
            {
                int centerX = (int)(data[0] * frame.cols);
                int centerY = (int)(data[1] * frame.rows);
                int width = (int)(data[2] * frame.cols);
                int height = (int)(data[3] * frame.rows);
                int left = centerX - width / 2;
                int top = centerY - height / 2;

                classIds.push_back(classIdPoint.x);
                confidences.push_back((float)confidence);
                boxes2.push_back(Rect(left, top, width, height));
            }
        }
    }

    // Perform non maximum suppression to eliminate redundant overlapping boxes with
    // lower confidences
    vector<int> indices;
    NMSBoxes(boxes2, confidences, confThreshold, nmsThreshold, indices);
    for (size_t i = 0; i < indices.size(); ++i)
    {
        int idx = indices[i];
        Rect box = boxes2[idx];
        drawPred(classIds[idx], confidences[idx], box.x, box.y,
                 box.x + box.width, box.y + box.height, frame, mode);
    }
}

void VehicleNode::publishVGAStereoImage(Vehicle* vehicle, RecvContainer recvFrame, DJI::OSDK::UserData userData)
{
  std::cout << "publishVGAStereoImage got called...\n";
  std::cout << "publishVGAStereoImage ggetting node_ptr...\n";
  VehicleNode *node_ptr = (VehicleNode *)userData;
  std::cout << "publishVGAStereoImage got called...\n";
  node_ptr->stereo_vga_subscription_success = true;
  std::cout << "stereo_vga_subscription_success set\n";

  sensor_msgs::Image img;
  img.height = 480;
  img.width = 640;
  img.step = 640;
  img.encoding = "mono8";
  img.data.resize(img.height*img.width);
  std::cout << "Image packet set\n";

  img.header.seq = recvFrame.recvData.stereoVGAImgData->frame_index;
  img.header.stamp = ros::Time::now(); // @todo
  img.header.frame_id = "vga_left";
  memcpy((char*)(&img.data[0]), recvFrame.recvData.stereoVGAImgData->img_vec[0], 480*640);
  std::cout << "memcpy left set\n";
  node_ptr->stereo_vga_front_left_publisher_.publish(img);

  img.header.frame_id = "vga_right";
  memcpy((char*)(&img.data[0]), recvFrame.recvData.stereoVGAImgData->img_vec[1], 480*640);
  std::cout << "memcpy right set\n";
  node_ptr->stereo_vga_front_right_publisher_.publish(img);
  std::cout << "publishVGAStereoImage done\n";
}


int tempX=0;
int tempY=0;
double agentVeloX=0;
double agentVeloY=0;
double targetVeloX=0;
double targetVeloY=0;
string trackerType;
std::fstream logFile;
double ppmx=-1;
double ppmy=-1;
double heightAboveGround=-1;
float fps=-1;

int sendCount=20;
void VehicleNode::publishMainCameraImage(CameraRGBImage rgbImg, void* userData)
{
  VehicleNode *node_ptr = (VehicleNode *)userData;
  //std::cout << "Publishing Image...\n";

  try{	
	Mat mat(rgbImg.height, rgbImg.width, CV_8UC3, rgbImg.rawData.data(), rgbImg.width*3);
  	//cvtColor(mat, mat, COLOR_BGR2RGB);
	//resize(mat, mat, Size(1280,720));
	//Mat im_tab = mat.clone();
    //Mat im_tab_540p = mat.clone();
  	//cvtColor(im_tab, im_tab, COLOR_RGB2BGR);

    Mat im_tab_720p;
    Mat im_tab_270p;

    resize(mat, im_tab_720p, node_ptr->resolution_1280_720);
    resize(mat, im_tab_270p, node_ptr->resolution_480_270);

    // Set the compression format
    std::string compression_format = "jpeg"; // You can also use "png" or "tiff"

    // Set the compression quality (0 to 100)
    int compression_quality = 90;

    //std::cout << "Image resized...\n";

	//std::vector<uchar> imgarray = node_ptr->mat2Imgarray(im_tab_720p);
	// if (im_tab.isContinuous()) {
    //     imgarray.assign(im_tab.data, im_tab.data + im_tab.total()*im_tab.channels());
	// } else {
    //     for (int i = 0; i < im_tab.rows; ++i) {
    //         imgarray.insert(imgarray.end(), im_tab.ptr<uchar>(i), im_tab.ptr<uchar>(i)+im_tab.cols*im_tab.channels());
    //     }
	// }
    //std::cout << "Image inserted to imgarray...\n";

    sensor_msgs::Image img;
    std::vector<uchar> imgarray_720p = node_ptr->mat2Imgarray(im_tab_720p);  
    img.height = node_ptr->resolution_1280_720.height;
    img.width = node_ptr->resolution_1280_720.width;
    img.step = node_ptr->resolution_1280_720.width * 3;

	img.encoding = "rgb8";
	//img.encoding = "mono8"; // grayscale image
	img.data = imgarray_720p;
	img.header.stamp = ros::Time::now();
	img.header.frame_id = "MAIN_CAMERA";

	node_ptr->main_camera_stream_publisher_.publish(img);

    std::vector<uchar> imgarray_270p = node_ptr->mat2Imgarray(im_tab_270p);  
    img.height = node_ptr->resolution_480_270.height;
    img.width = node_ptr->resolution_480_270.width;
    img.step = node_ptr->resolution_480_270.width * 3;

    img.data = imgarray_270p;
	img.header.stamp = ros::Time::now();

    node_ptr->main_camera_stream_270p30fps_publisher_.publish(img);
    
    
    
    
    //std::cout << "Image Published\n";
	/*}
	sendCount++;
	if(sendCount==21)
		sendCount=1;
// KALMAN
//        try{
//		bool useKalman=true;
//		ifstream kalmanFile;
//	   	kalmanFile.open("/home/jetson/catkin_ws/kalmanOrNot.txt");
//	   	if(!kalmanFile){
//			useKalman=true;
//	   	}else
//	       		kalmanFile >> useKalman;
//	   	kalmanFile.close();
//		if(useKalman){
//		    node_ptr->detectAndTrack(rgbImg, userData);
//		}else{
//		    Mat mat(rgbImg.height, rgbImg.width, CV_8UC3, rgbImg.rawData.data(), rgbImg.width*3);
//		    cvtColor(mat, mat, COLOR_RGB2BGR);
//		    int width=1280;
//		    int height=720;
//	    	    resize(mat, mat, cvSize(width, height), cv::INTER_LINEAR);
//		    im = mat.clone();
//		    static const string kWinName = "OpenCV Tracker - Main Camera";
//		    namedWindow(kWinName, WINDOW_AUTOSIZE);

//		    if(!findTargetId){
//			    if(!videoOut){
//				    // List of tracker types in OpenCV 3.4.1
//				    string trackerTypes[8] = {"BOOSTING", "MIL", "KCF", "TLD","MEDIANFLOW", "GOTURN", "MOSSE", "CSRT"};
//				    int type;
//				    ifstream trackerFile;
//				    trackerFile.open("/home/jetson/catkin_ws/trackerType.txt");
//				    if(!trackerFile){
//				    	type=2;
//				    }else
//				    	trackerFile >> type;
//				    trackerFile.close();
//				    trackerType = trackerTypes[type];
//				    // define video file pathname
//				    boost::posix_time::ptime my_time = ros::Time::now().toBoost();
//				    std::string time_str = boost::posix_time::to_iso_string(my_time);
//				    std::string videoFilePath = "/home/jetson/catkin_ws/tracking/trackVideo"+trackerType+"_"+time_str.substr(0, 15)+".mkv";
//				    videoWriter = cv::VideoWriter(videoFilePath, cv::VideoWriter::fourcc('x','2','6','4'), 25.0, cv::Size(width, height), true);
//				    videoOut=true;
		
//				    // define data logfile pathname
//				    std::string logFilePath = "/home/jetson/catkin_ws/tracking/trackLOG"+trackerType+"_"+time_str.substr(0, 15)+".csv";
//				    logFile.open(logFilePath, ios::out | ios::app);
//				    logFile << "Timestamp,Frame Num,Target X,Target Y,Target W,Target H,PPMX,PPMY,Altitude,FPS,Control Speed X, Control Speed Y, Target Velo X, Target Velo Y, Agent Velo X, Agent Velo Y\n";
//				    logFile << std::fixed << std::setprecision(10);

//				    std::cout << std::endl << "---------- INITIALIZING TRACKER! ----------" << std::endl;
//				    VehicleNode::getTracker()->clear();
//				    VehicleNode::getTracker()->init(mat, bbox);
//		     	    }else{
//				    // Start timer
//				    double timer = (double)getTickCount();
				
//				    Rect2d prev_bbox = bbox;
//				    // Update the tracking result
//				    bool ok = VehicleNode::getTracker()->update(mat, bbox);
				
//				    // Calculate Frames per second (FPS)
//				    fps = getTickFrequency() / ((double)getTickCount() - timer);
				
//				    if (ok)
//				    {
//				       double targetVeloX = (bbox.x - prev_bbox.x)*fps;
//				       double targetVeloY = (bbox.y - prev_bbox.y)*fps;
//				       std::cout << std::endl << "-------- Send Track --------" << std::endl;
//				       node_ptr->trackCar(bbox.x+40, bbox.y+40, targetVeloX, targetVeloY, width, height);

//				       // Tracking success : Draw the tracked object
//				       //rectangle(mat, bbox, Scalar( 255, 0, 0 ), 2, 1 );

//				       std::cout << std::endl << "-------- Constructing Tracker's ROS Bounding Box --------" << std::endl;
//				       dji_sdk::BoundingBoxes boundBoxes;
//				       boundBoxes.header.stamp = ros::Time::now();
//				       boundBoxes.header.frame_id = "MAIN_CAMERA";
//				       boundBoxes.labels.resize(1);
//				       boundBoxes.confidences.resize(1);
//				       boundBoxes.tops.resize(1);
//				       boundBoxes.lefts.resize(1);
//				       boundBoxes.rights.resize(1);
//				       boundBoxes.bottoms.resize(1);
//				       boundBoxes.labels[0] = trackerType;
//				       boundBoxes.confidences[0] = 1;
//				       boundBoxes.tops[0] = (bbox.x) / (width * 1.0);
//				       boundBoxes.lefts[0] = (bbox.y) / (height * 1.0);
//				       boundBoxes.rights[0] = (bbox.x + 80) / (width * 1.0);
//				       boundBoxes.bottoms[0] = (bbox.y + 80) / (height * 1.0);
//				       std::cout << std::endl << "-------- Sending Tracker's ROS Bounding Box --------" << std::endl;
//				       node_ptr->bounding_boxes_publisher_.publish(boundBoxes);
				       
//				   }
//				   else
//				   {
//				       dji_sdk::BoundingBoxes boundBoxes;
//				       boundBoxes.header.stamp = ros::Time::now();
//				       boundBoxes.header.frame_id = "MAIN_CAMERA";
//				       boundBoxes.labels.resize(0);
//				       boundBoxes.confidences.resize(0);
//				       boundBoxes.tops.resize(0);
//				       boundBoxes.lefts.resize(0);
//				       boundBoxes.rights.resize(0);
//				       boundBoxes.bottoms.resize(0);
//				       node_ptr->bounding_boxes_publisher_.publish(boundBoxes);

//				       node_ptr->trackCar(width/2, height/2, 0.0, 0.0, width, height);
//				       // Tracking failure detected.
//				       //putText(mat, "Tracking failed", Point(100,80), FONT_HERSHEY_TRIPLEX, 0.75, Scalar(0,0,255),2);
//				   }
//			   	   // Display tracker type on frame
//			   	   //putText(mat, "Tracker Type: " + trackerType, Point(100,20), FONT_HERSHEY_TRIPLEX, 0.75, Scalar(50,170,50),2);
			
//			   	   // Display FPS on frame
//			   	   //string string_fps = format("FPS : %.2f", fps);
//			   	   //putText(mat, string_fps, Point(100,50), FONT_HERSHEY_TRIPLEX, 0.75, Scalar(50,170,50), 2);

//			   	   //put copyright data on the frame
//			   	   string copyright = format("(c) KIOS CoE");
//			   	   putText(mat, copyright, Point(width-200, height-20), FONT_HERSHEY_TRIPLEX, 0.75, Scalar(255,0,255),2);

//				   //write frame to video out
//				   videoWriter.write(mat);
//			   }
//		   }
			
//		   VehicleNode::getTracker()->clear();
//		   dji_sdk::BoundingBoxes boundBoxes;
//		   boundBoxes.header.stamp = ros::Time::now();
//		   boundBoxes.header.frame_id = "MAIN_CAMERA";
//		   boundBoxes.labels.resize(0);
//		   boundBoxes.confidences.resize(0);
//		   boundBoxes.tops.resize(0);
//		   boundBoxes.lefts.resize(0);
//		   boundBoxes.rights.resize(0);
//		   boundBoxes.bottoms.resize(0);
	
//		   // add cross lines at the center of the frame
//	  	   //line(mat, Point(width/2, (height/2)-30), Point(width/2, (height/2)+30), Scalar(255,0,255), 1);
//		   //line(mat, Point((width/2)-30, height/2), Point((width/2)+30, height/2), Scalar(255,0,255), 1);

//		   // Display frame.
//		   imshow(kWinName, mat);
//		   waitKey(1);

//		   // add line to log file
//		   boost::posix_time::ptime my_time = ros::Time::now().toBoost();
//		   std::string time_str = boost::posix_time::to_iso_string(my_time);
//		   logFile << time_str.substr(0, 20) << "," << frame_num << "," << bbox.x << "," << bbox.y << "," << bbox.width << "," << bbox.height << "," << ppmx << "," << ppmy << "," << heightAboveGround << "," << fps << "," << flightControlData.x << "," << flightControlData.y << "," << targetVeloX << "," << targetVeloX << "," << agentVeloX << "," << agentVeloX << "\n";
//		}
//		set_frame_id(++frame_num);

//        }catch (cv::Exception e){
//            cout << "CV Exception caught: " << e.what() << endl;
//        }catch (std::bad_alloc e){
//            cout << "std Exception caught: " << e.what() << endl;
//        }catch(std::length_error e){
//            cout << "std Exception caught: " << e.what() << endl;
//        }
//	//std::cout << std::endl << "-------- Before clear actives --------" << std::endl;
//        actives.clear();
//        classIds.clear();
//	if(sModeTriggered){
//		node_ptr->stopDetector();
//		sModeTriggered = false;
//	}*/
//	//std::cout << std::endl << "-------- After clear actives --------" << std::endl;
    }catch(std::exception& e){
        cout << "Exception caught: " << e.what() << endl;
    }
}


std::vector<uchar> VehicleNode::mat2Imgarray(Mat im_tab) {

	std::vector<uchar> imgarray;
	if (im_tab.isContinuous()) {
        imgarray.assign(im_tab.data, im_tab.data + im_tab.total()*im_tab.channels());
	} else {
        for (int i = 0; i < im_tab.rows; ++i) {
            imgarray.insert(imgarray.end(), im_tab.ptr<uchar>(i), im_tab.ptr<uchar>(i)+im_tab.cols*im_tab.channels());
        }
	}

    return imgarray;
}


void VehicleNode::publishPullMainCameraImage()
{
  Vehicle* vehicle = ptr_wrapper_->getVehicle();
  CameraRGBImage mainImg;
  
  if(vehicle->advancedSensing->newMainCameraImageReady())
	if(vehicle->advancedSensing->getMainCameraImage(mainImg)){
		Mat mat(mainImg.height, mainImg.width, CV_8UC3, mainImg.rawData.data(), mainImg.width*3);
		cvtColor(mat, mat, COLOR_RGB2BGR);
		imshow("Test", mat);
		waitKey(1);
	}
  //usleep(2e4);
}
/*
void VehicleNode::onClickCallback(int event, int x, int y, int flags, void* userData){
	VehicleNode *node_ptr = reinterpret_cast<VehicleNode*>(userData);
 	if (event == 4){
		node_ptr->nearestBox(x/1280.0, y/720.0);
	}else if(event == 6)
	{
		node_ptr->stopDetector();
	}else if(event == 10){
		GimbalRotationData gimbalRotationData;
    		gimbalRotationData.rotationMode = 0;
    		gimbalRotationData.pitch = x;
    		gimbalRotationData.roll  = 0;
    		gimbalRotationData.yaw   = 0;
   		gimbalRotationData.time  = 0;
   		bool gimbal_result = node_ptr->rotateGimbal(static_cast<PayloadIndex>(0), gimbalRotationData);
	}
}*/

int boundingBoxCounter=0;
int boundingBoxCounterModulo=5;
void VehicleNode::detectAndTrack(CameraRGBImage rgbImg, void* userData)
{

    VehicleNode *node_ptr = (VehicleNode *)userData;
    auto start = std::chrono::system_clock::now();
    Net net = VehicleNode::getNetworkHarpy();

    Mat blob;
    Mat mat(rgbImg.height, rgbImg.width, CV_8UC3, rgbImg.rawData.data(), rgbImg.width*3);
	
    cvtColor(mat, mat, COLOR_RGB2BGR);
    int width=1280;
    int height=720;
    resize(mat, mat, cvSize(width, height), cv::INTER_LINEAR);
    im = mat.clone();
    // Create a window
    static const string kWinName = "Deep learning object detection - Main Camera";
    namedWindow(kWinName, WINDOW_AUTOSIZE);
   //cv::setMouseCallback(kWinName, onClickCallback, node_ptr);
    //imshow(kWinName, mat);
    //waitKey(1);
    // Create a 4D blob from a frame.
    blobFromImage(mat, blob, 1/255.0, cvSize(416, 416), Scalar(0,0,0), true, false);

    //Sets the input to the network
    net.setInput(blob);

    // Runs the forward pass to get output of the output layers
    vector<Mat> outs;
    //static vector<String> outputNames = getOutputsNames(net);
    net.forward(outs, getOutputsNames(net));

    if(outs.size() < 1)
        return;

    //std::cout << std::endl << "-------- Before postprocess --------" << std::endl;
    // Remove the bounding boxes with low confidence
    postprocess(mat, outs);
    //std::cout << std::endl << "-------- After postprocess --------" << std::endl;

    // Put efficiency information. The function getPerfProfile returns the overall time for inference(t) and the timings for each of the layers(in layersTimes)
    //        vector<double> layersTimes;
    //        double freq = getTickFrequency() / 1000;
    //        double t = net.getPerfProfile(layersTimes) / freq;
    //        string label = format("Inference time for a frame : %.2f ms", t);
    //        putText(mat, label, Point(0, 15), FONT_HERSHEY_TRIPLEX, 0.5, Scalar(0, 0, 255));

    dji_sdk::BoundingBoxes boundBoxes;
    boundBoxes.header.stamp = ros::Time::now();
    boundBoxes.header.frame_id = "MAIN_CAMERA";
    vector<string> classes = VehicleNode::getClassesHarpy();

    if(!findTargetId){

    	if(!videoOut){
		boost::posix_time::ptime my_time = ros::Time::now().toBoost();
		std::string time_str = boost::posix_time::to_iso_string(my_time);
		std::string videoFilePath = "/home/jetson/catkin_ws/tracking/trackVideoHyperion_"+time_str.substr(0, 15)+".mkv";
		videoWriter = cv::VideoWriter(videoFilePath, cv::VideoWriter::fourcc('x','2','6','4'), 20.0, cv::Size(width, height), true);
		videoOut=true;

		// define data logfile pathname
		std::string logFilePath = "/home/jetson/catkin_ws/tracking/trackLOGHyperion_"+time_str.substr(0, 15)+".csv";
		logFile.open(logFilePath, ios::out | ios::app);
		logFile << "Timestamp,Frame Num,Target X,Target Y,Target W,Target H,PPMX,PPMY,Altitude,FPS, Control Speed X, Control Speed Y, Target Velo X, Target Velo Y, Agent Velo X, Agent Velo Y\n";
		logFile << std::fixed << std::setprecision(10);
     	}

        Eigen::VectorXd y(2);
        Eigen::MatrixXd A(4, 4); // System dynamics matrix
        A << 1, 0, dt, 0, 0, 1, 0, dt, 0, 0, 1, 0, 0, 0, 0, 1;

        int lastb=cars[targetId].b.size()-1;
        float predX, predY, predXdot, predYdot;

        bool stillActive = false;
        for(int k=0; k < actives.size(); k++){
            if(actives[k] == targetId)
                stillActive = true;
        }

        if (!stillActive){
	    //std::cout << std::endl << "-------- target not active Kalman SELF update --------" << std::endl;
            Eigen::VectorXd prediction = VehicleNode::getKalmanFilter()->state().transpose();
            predX = prediction[0];
            predY = prediction[1];
            predXdot = prediction[2];
            predYdot = prediction[3];
	    
	    // in case kalman gets out of image boundaries
  	    if((predX-averageTargetBox.w/2.0)<0.0){
		predX = averageTargetBox.w/2.0+10;
	    }
	    else if ((predX+averageTargetBox.w/2.0)>width) {
	    	predX = width - averageTargetBox.w/2.0-10;
	    }
  	    if((predY-averageTargetBox.h/2.0)<0.0){
		predY = averageTargetBox.h/2.0+10;
	    }
	    else if ((predY+averageTargetBox.h/2.0)>height) {
	    	predY = height - averageTargetBox.h/2.0-10;
	    }

            cars[targetId].b[lastb].x = predX;
            cars[targetId].b[lastb].y = predY;
	    cars[targetId].active = false;

	    frame_indicate[targetId].push_back(frame_num-1);
            
	    y << (predX+(predXdot*dt)), (predY+(predYdot*dt));
	    //y << predX, predY;

            VehicleNode::getKalmanFilter()->update(y, dt, A);

        }else{
	    //std::cout << std::endl << "-------- target active Kalman VISION update --------" << std::endl;
            y << cars[targetId].b[lastb].x, cars[targetId].b[lastb].y;
	    //y << targetVeloX, targetVeloY;
	    // Update the tracking result
	    //bool ok = tracker->update(mat, bbox);
            VehicleNode::getKalmanFilter()->update(y, dt, A);

            Eigen::VectorXd prediction = VehicleNode::getKalmanFilter()->state().transpose();
            predX = prediction[0];
            predY = prediction[1];
            predXdot = prediction[2];
            predYdot = prediction[3];

        }
	if(boundingBoxCounter%boundingBoxCounterModulo == 0){
		//std::cout << std::endl << "-------- Set Bounding Boxes for ROS --------" << std::endl;
		boundBoxes.labels.resize(actives.size()+1);
		boundBoxes.confidences.resize(actives.size()+1);
		boundBoxes.tops.resize(actives.size()+1);
		boundBoxes.lefts.resize(actives.size()+1);
		boundBoxes.rights.resize(actives.size()+1);
		boundBoxes.bottoms.resize(actives.size()+1);
		for (int detections=0; detections < actives.size(); detections++){
		    int lastb=cars[actives[detections]].b.size()-1;
		    boundBoxes.labels[detections] = classes[classIds[detections]];
		    boundBoxes.confidences[detections] = cars[actives[detections]].prob;
		    boundBoxes.tops[detections] = (cars[actives[detections]].b[lastb].x - cars[actives[detections]].b[lastb].w/2) / (width * 1.0);
		    boundBoxes.lefts[detections] = (cars[actives[detections]].b[lastb].y - cars[actives[detections]].b[lastb].h/2) / (height * 1.0);
		    boundBoxes.rights[detections] = (cars[actives[detections]].b[lastb].x + cars[actives[detections]].b[lastb].w/2) / (width * 1.0);
		    boundBoxes.bottoms[detections] = (cars[actives[detections]].b[lastb].y + cars[actives[detections]].b[lastb].h/2) / (height * 1.0);

		    String boundingBoxString = to_string((int)(cars[actives[detections]].b[lastb].x-cars[actives[detections]].b[lastb].w/2)*1000/(width))+","+ to_string((int)(cars[actives[detections]].b[lastb].y-cars[actives[detections]].b[lastb].h/2)*1000/(height)) +","+ to_string((int)(cars[actives[detections]].b[lastb].x+cars[actives[detections]].b[lastb].w/2)*1000/(width)) +","+ to_string((int)(cars[actives[detections]].b[lastb].y+cars[actives[detections]].b[lastb].h/2)*1000/(height))+"," + to_string(1) + ",";
		    //std::cout<< boundingBoxString << std::endl;
			
		    std::vector<unsigned char> bytes(boundingBoxString.begin(), boundingBoxString.end());
		    bytes.push_back('\0');
		
		    dji_sdk::SendMobileData request;
		    request.request.data = bytes;
		    node_ptr->send_data_to_mobile_device_client_.call(request);
		}
	}
	

	Point topl = Point(predX - averageTargetBox.w/2-3, predY - averageTargetBox.h/2-3);
	Point botr = Point(predX + averageTargetBox.w/2+3, predY + averageTargetBox.h/2+3);
	Scalar color= CV_RGB(255,0,0); // #1 box is red
	Point text_lbl = Point(topl.x -5,topl.y -5);
	rectangle(mat,topl,botr, color, 2);
	string textlbl =to_string(cars[targetId].id) + " Hyperion";//to_string(cars[actives[idx]].id);
	putText(mat, textlbl,text_lbl, FONT_HERSHEY_TRIPLEX, 1.1, Scalar(0,0,255));

	if(boundingBoxCounter%boundingBoxCounterModulo == 0){
		boundBoxes.labels[actives.size()] = "Target";
		boundBoxes.confidences[actives.size()] = cars[targetId].prob;
		boundBoxes.tops[actives.size()] = ((cars[targetId].b[lastb].x - averageTargetBox.w/2) -10) / (width * 1.0);
		boundBoxes.lefts[actives.size()] = ((cars[targetId].b[lastb].y - averageTargetBox.h/2) -10) / (height * 1.0);
		boundBoxes.rights[actives.size()] = ((cars[targetId].b[lastb].x + averageTargetBox.w/2) +10) / (width * 1.0);
		boundBoxes.bottoms[actives.size()] = ((cars[targetId].b[lastb].y + averageTargetBox.h/2) +10) / (height * 1.0);
	
		String boundingBoxString = to_string((int)((cars[targetId].b[lastb].x-averageTargetBox.w/2)-10)*1000/(width))+","+ to_string((int)((cars[targetId].b[lastb].y-averageTargetBox.h/2)-10)*1000/(height)) +","+ to_string((int)((cars[targetId].b[lastb].x+averageTargetBox.w/2)+10)*1000/(width)) +","+ to_string((int)((cars[targetId].b[lastb].y+averageTargetBox.h/2)+10)*1000/(height))+"," + to_string(2) + ",";
		//std::cout<< boundingBoxString << std::endl;
			
		std::vector<unsigned char> bytes(boundingBoxString.begin(), boundingBoxString.end());
		bytes.push_back('\0');
			
		dji_sdk::SendMobileData request;
		request.request.data = bytes;
		node_ptr->send_data_to_mobile_device_client_.call(request);

		boundingBoxString = to_string(254);
		std::vector<unsigned char> bytes2(boundingBoxString.begin(), boundingBoxString.end());
		bytes2.push_back('\0');
		//std::cout<< boundingBoxString << std::endl;

		request.request.data = bytes2;
		node_ptr->send_data_to_mobile_device_client_.call(request);        
		node_ptr->bounding_boxes_publisher_.publish(boundBoxes);
	}

	boundingBoxCounter++;
	if(boundingBoxCounter>=1000)
		boundingBoxCounter=0;
        
	if(tempX == 0 && tempY){
            tempX = cars[targetId].b[lastb].x;
            tempY = cars[targetId].b[lastb].y;
        }

        double targetVeloX = (cars[targetId].b[lastb].x - tempX)/dt;
        double targetVeloY = (cars[targetId].b[lastb].y - tempY)/dt;
	//std::cout << std::endl << "-------- Send Track --------" << std::endl;
        node_ptr->trackCar(cars[targetId].b[lastb].x, cars[targetId].b[lastb].y, targetVeloX, targetVeloY, width, height);
        tempX = cars[targetId].b[lastb].x;
        tempY = cars[targetId].b[lastb].y;



       // put the fps count on the frame
        auto finish = std::chrono::system_clock::now();
        dt = std::chrono::duration_cast<std::chrono::duration<double>>(finish-start).count();
        string fps = format("FPS : %.2f", 1/dt);
	cv::Size textSizeFps = getTextSize(fps, FONT_HERSHEY_TRIPLEX, 1.5, 2, 0);
        rectangle(mat, Point(textSizeFps.width+5, 40), Point(0, 30-textSizeFps.height), Scalar(0,0,0),-1);

	//put the drone velocities and altitude on the frame
   	string velos = format("UAV Velocity X: %.2f, UAV Velocity Y: %.2f, UAV Altitude: %.2f", agentVeloX, agentVeloY, heightAboveGround);
	cv::Size textSizeVelos = getTextSize(velos, FONT_HERSHEY_TRIPLEX, 0.7, 1.5, 0);
        rectangle(mat, Point(textSizeVelos.width+5, 60), Point(0, 50-textSizeVelos.height), Scalar(0,0,0),-1);
        putText(mat, velos, Point(0, 55), FONT_HERSHEY_TRIPLEX, 0.7, Scalar(255,255,255),1.5);
        putText(mat, fps, Point(0, 35), FONT_HERSHEY_TRIPLEX, 1.5, Scalar(255,255,255), 2);


	//put copyright data on the frame
   	string copyright = format("(c) KIOS CoE");
        cv::Size textSizeCopyright = getTextSize(copyright, FONT_HERSHEY_TRIPLEX, 0.75, 2, 0);
        rectangle(mat, Point(width, height), Point(width-textSizeCopyright.width-10, height-textSizeCopyright.height-15), Scalar(0,0,0),-1);
        putText(mat, copyright, Point(width-textSizeCopyright.width-5, height-10), FONT_HERSHEY_TRIPLEX, 0.75, Scalar(255,255,255),2);

	line(mat, Point(cars[targetId].b[lastb].x, cars[targetId].b[lastb].y-15), Point(cars[targetId].b[lastb].x, cars[targetId].b[lastb].y+15), Scalar(0,0,255), 2);
        line(mat, Point(cars[targetId].b[lastb].x-15, cars[targetId].b[lastb].y), Point(cars[targetId].b[lastb].x+15, cars[targetId].b[lastb].y), Scalar(0,0,255), 2);
        line(mat, Point(cars[targetId].b[lastb].x, cars[targetId].b[lastb].y), Point((width/2),(height/2)), Scalar(0,0,255), 2);
        rectangle(mat,Point((width/2)-30, (height/2)-30),Point((width/2)+30, (height/2)+30), Scalar(0,0,255),2);
	//line(mat, Point(width/2, (height/2)-30), Point(width/2, (height/2)+30), Scalar(255,0,255), 1);
        //line(mat, Point((width/2)-30, height/2), Point((width/2)+30, height/2), Scalar(255,0,255), 1);


        // add line to log file
	boost::posix_time::ptime my_time = ros::Time::now().toBoost();
	std::string time_str = boost::posix_time::to_iso_string(my_time);
	double targetTopX = cars[targetId].b[lastb].x - cars[targetId].b[lastb].w/2.0;
	double targetTopY = cars[targetId].b[lastb].y - cars[targetId].b[lastb].h/2.0;
	logFile << time_str.substr(0, 20) << "," << frame_num << "," << targetTopX << "," << targetTopY << "," << cars[targetId].b[lastb].w << "," << cars[targetId].b[lastb].h << "," << ppmx << "," << ppmy << "," << heightAboveGround << "," << 1/dt << "," << flightControlData.x << "," << flightControlData.y << "," << targetVeloX << "," << targetVeloX << "," << agentVeloX << "," << agentVeloX << "\n";

        //show the frame
        imshow(kWinName, mat);
        waitKey(1);

        videoWriter.write(mat);
	//std::cout << std::endl << "-------- Finish Track --------" << std::endl;

	
	
    }else{


	if(boundingBoxCounter%boundingBoxCounterModulo == 0){
		boundBoxes.labels.resize(actives.size());
		boundBoxes.confidences.resize(actives.size());
		boundBoxes.tops.resize(actives.size());
		boundBoxes.lefts.resize(actives.size());
		boundBoxes.rights.resize(actives.size());
		boundBoxes.bottoms.resize(actives.size());
		for (int detections=0; detections < actives.size(); detections++){
		    int lastb=cars[actives[detections]].b.size()-1;
		    boundBoxes.labels[detections] = classes[classIds[detections]];
		    boundBoxes.confidences[detections] = cars[actives[detections]].prob;
		    boundBoxes.tops[detections] = (cars[actives[detections]].b[lastb].x - cars[actives[detections]].b[lastb].w/2) / (width * 1.0);
		    boundBoxes.lefts[detections] = (cars[actives[detections]].b[lastb].y - cars[actives[detections]].b[lastb].h/2) / (height * 1.0);
		    boundBoxes.rights[detections] = (cars[actives[detections]].b[lastb].x + cars[actives[detections]].b[lastb].w/2) / (width * 1.0);
		    boundBoxes.bottoms[detections] = (cars[actives[detections]].b[lastb].y + cars[actives[detections]].b[lastb].h/2) / (height * 1.0);

		    String boundingBoxString = to_string((int)(cars[actives[detections]].b[lastb].x-cars[actives[detections]].b[lastb].w/2)*1000/(width))+","+ to_string((int)(cars[actives[detections]].b[lastb].y-cars[actives[detections]].b[lastb].h/2)*1000/(height)) +","+ to_string((int)(cars[actives[detections]].b[lastb].x+cars[actives[detections]].b[lastb].w/2)*1000/(width)) +","+ to_string((int)(cars[actives[detections]].b[lastb].y+cars[actives[detections]].b[lastb].h/2)*1000/(height))+"," + to_string(1) + ",";
		//std::cout<< boundingBoxString << std::endl;
			
		    std::vector<unsigned char> bytes(boundingBoxString.begin(), boundingBoxString.end());
		    bytes.push_back('\0');
		
		    dji_sdk::SendMobileData request;
		    request.request.data = bytes;
		    node_ptr->send_data_to_mobile_device_client_.call(request);
		}
	

		String boundingBoxString = to_string(254);
		//std::cout<< boundingBoxString << std::endl;
			
		std::vector<unsigned char> bytes(boundingBoxString.begin(), boundingBoxString.end());
		bytes.push_back('\0');
		
		dji_sdk::SendMobileData request;
		request.request.data = bytes;
		node_ptr->send_data_to_mobile_device_client_.call(request);

		node_ptr->bounding_boxes_publisher_.publish(boundBoxes);
	}
	boundingBoxCounter++;
	if(boundingBoxCounter>=1000)
		boundingBoxCounter=0;
	// put the fps count on the frame
        auto finish = std::chrono::system_clock::now();
        dt = std::chrono::duration_cast<std::chrono::duration<double>>(finish-start).count();
        string fps = format("FPS : %.2f", 1/dt);
        //putText(mat, fps, Point(0, 35), FONT_HERSHEY_TRIPLEX, 1.5, Scalar(0, 0, 255), 2);
        
	//put the target velocities on the frame
   	//string velos = format("Target Velo X: %.2f, Target Velo Y: %.2f", targetVeloX, targetVeloY);
    	//putText(mat, velos, Point(0, 50), FONT_HERSHEY_TRIPLEX, 0.5, Scalar(0, 0, 255));

	//put copyright data on the frame
   	string copyright = format("(c) KIOS CoE");
   	cv::Size textSizeCopyright = getTextSize(copyright, FONT_HERSHEY_TRIPLEX, 0.75, 2, 0);
        rectangle(mat, Point(width, height), Point(width-textSizeCopyright.width-10, height-textSizeCopyright.height-15), Scalar(0,0,0),-1);
        putText(mat, copyright, Point(width-textSizeCopyright.width-5, height-10), FONT_HERSHEY_TRIPLEX, 0.75, Scalar(255,255,255),2);


	//line(mat, Point(width/2, (height/2)-30), Point(width/2, (height/2)+30), Scalar(255,0,255), 1);
        //line(mat, Point((width/2)-30, height/2), Point((width/2)+30, height/2), Scalar(255,0,255), 1);

        //show the frame
        imshow(kWinName, mat);
        waitKey(1);
    }
}

void VehicleNode::trackCar(int x, int y, float xdot, float ydot, int width, int height)
{
    Vehicle* vehicle = ptr_wrapper_->getVehicle();
    if(!hasAuthority){
        ACK::ErrorCode ack;
	std::cout << std::endl << "---------- Obtaining Control Authority ----------" << std::endl;
        ack = vehicle->control->obtainCtrlAuthority(2);
        if (ACK::getError(ack)){
	    std::cout << std::endl << "---------- NOT Able To Obtain Control Authority ----------" << std::endl;
            hasAuthority = false;
        }else{
	    std::cout << std::endl << "---------- Control Authority Obtained ----------" << std::endl;
            hasAuthority = true;
	}
    }

    std::cout << std::setprecision(6) << std::fixed;

    //    double siny_cosp = 2.0 * (quaternion2.getW() * quaternion2.getZ() + quaternion2.getX() * quaternion2.getY());
    //    double cosy_cosp = 1.0 - 2.0 * (quaternion2.getY() * quaternion2.getY() + quaternion2.getZ() * quaternion2.getZ());
    //    double yaw = atan2(cosy_cosp, siny_cosp) * 180.0 / C_PI;
    //    double targetYaw = yaw - ((atan2((height - y),(x - width/2)) * 180.0 / C_PI)-90.0);

    //    if(targetYaw < -180.0)
    //        targetYaw += 360.0;
    //    else if(targetYaw > 180.0)
    //        targetYaw -= 360.0;

    //    if(abs(targetYaw-yaw) > 10.0)
    //        flightControlData.yaw = targetYaw;
    //    else
    flightControlData.yaw = 0;


    //    std::cout<< "Current Yaw: " << yaw << " Target Yaw: " << targetYaw  << " Vision Angle: " <<  (-((atan2(y,x) * 180.0 / C_PI)-90.0)) << std::endl;

    Telemetry::TypeMap<Telemetry::TOPIC_ALTITUDE_FUSIONED>::type fused_altitude =
            vehicle->subscribe->getValue<Telemetry::TOPIC_ALTITUDE_FUSIONED>();


    //ry = 2 * altitude * tan(VFOV/2)
    //rx = 2 * altitude * tan(HFOV/2)	
    double ry = 0.0;
    double rx = 0.0;

    heightAboveGround = (fused_altitude-homepointHeight);

    if(vehicle->isM300()){
	//std::cout << std::endl << "---------- Setting RX and RY for H20T ZOOM ----------" << std::endl;
	//H20T zoom
	ry = (fused_altitude-homepointHeight) * 2.0 * tan(21.087067/2.0*(C_PI/180.0));
	rx = (fused_altitude-homepointHeight) * 2.0 * tan(37.04976/2.0*(C_PI/180.0));
	//cout<<"PIXEL PER METER"<<ry<<","<<rx<<","<<tan(21.087067/2.0*(C_PI/180.0))<<endl;
    }else{
	//X4S
	ry = (fused_altitude-homepointHeight) * 0.87482;
	rx = (fused_altitude-homepointHeight) * 1.8;
    }

    //XT2
    //double ry = (fused_altitude-homepointHeight)*1.523706;
    //double rx = (fused_altitude-homepointHeight)*1.535797;

    ppmy = height/ry;
    ppmx = width/rx;

    //std::cout << std::endl << "---------- Calculating Target's Speed ----------" << std::endl;
    Telemetry::TypeMap<Telemetry::TOPIC_VELOCITY>::type v_FC =
            vehicle->subscribe->getValue<Telemetry::TOPIC_VELOCITY>();

    double siny_cosp = 2.0 * (quaternion2.getW() * quaternion2.getZ() + quaternion2.getX() * quaternion2.getY());
    double cosy_cosp = 1.0 - 2.0 * (quaternion2.getY() * quaternion2.getY() + quaternion2.getZ() * quaternion2.getZ());
    double yaw = atan2(cosy_cosp, siny_cosp);


    agentVeloX    =  v_FC.data.x*cos(yaw) + v_FC.data.y*sin(yaw);  //x, y are swapped from NE to ENs
    agentVeloY    =  (-1.0)*v_FC.data.x*sin(yaw) + v_FC.data.y*cos(yaw);

    //std::cout << " agentVeloX: " << agentVeloX << std::endl;
    //std::cout << " agentVeloY: " << agentVeloY << std::endl;

    double visionVeloX = ((-ydot/1.0)/ppmy);
    double visionVeloY = ((xdot/1.0)/ppmx);
    targetVeloX = agentVeloX + visionVeloX;
    targetVeloY = agentVeloY + visionVeloY;

    //std::cout << " targetVeloX: " << targetVeloX << std::endl;
    //std::cout << " targetVeloY: " << targetVeloY << std::endl;

    //std::cout << std::endl << "---------- Calculating PID Speed ----------" << std::endl;

    //std::cout << "Photo height: " << height << "Photo width: " << width <<  std::endl;
    //std::cout << "target x: " << x << "target y: " << y <<  std::endl;

    
    double pidVeloX= 0;
    pidVeloX = getPIDx()->calculate(height/2.0, targetVeloX, y * (1.0), agentVeloX);
    //std::cout << " pidVeloX: " << pidVeloX << std::endl;

    double pidVeloY = 0;
    pidVeloY = getPIDy()->calculate(width/2.0, targetVeloY, x * (1.0), agentVeloY);
    //std::cout << " pidVeloY: " << pidVeloY << std::endl;
    pidVeloY = (-1.0)*pidVeloY;

    double newVeloX = pidVeloX;// + targetVeloX;
    double newVeloY = pidVeloY;// + targetVeloY;

    //Linear controller insted of PID (only for debuging)
    //double newVeloY = (x-(width/2.0))*(20.0/(width/2.0));
    //double newVeloX  = (-1.0)*(y-(height/2.0))*(20.0/(height/2.0));

    //std::cout << " newVeloX: " << newVeloX << std::endl;
    //std::cout << " newVeloY: " << newVeloY << std::endl;


    //std::cout << std::endl << "---------- Setting New Desired Speed ----------" << std::endl;
    if(abs(width/2-x) >= 1.5*ppmx)
	  if(pidVeloY<=0.5)
		flightControlData.y = pidVeloY+targetVeloY;
	  else
	  	flightControlData.y = pidVeloY;
//        flightControlData.y = (flightControlData.y + newVeloY)/2.0;
    else
        flightControlData.y = 0;

    if(abs(height/2-y) >= 1.5*ppmy)
	  if(pidVeloX<=0.5)
		flightControlData.x = pidVeloX+targetVeloX;
	  else
	  	flightControlData.x = pidVeloX;
        //flightControlData.x = (flightControlData.x + newVeloX)/2.0;
    else
        flightControlData.x = 0;

    if(rc_data_msg.axes.at(3)!=NULL)
    	flightControlData.z = rc_data_msg.axes.at(3)*2.5;

    //std::cout << "VisionX: "  << visionVeloX << " VisionY: "   << visionVeloY << std::endl;
    //std::cout << "pidVeloX: " << pidVeloX    << " new VeloX: " << newVeloX    << " FlightCommandX: " << flightControlData.x << std::endl;
    //std::cout << "pidVeloY: " << pidVeloY    << " new VeloY: " << newVeloY    << " FlightCommandY: " << flightControlData.y << std::endl;

    
    //    if(fileName.empty()){
    //        boost::posix_time::ptime my_time = ros::Time::now().toBoost();
    //        time_str = boost::posix_time::to_iso_string(my_time);
    //        fileName =  "/home/jetson/catkin_ws/trackCar" + time_str.substr(0, 15) + ".txt";
    //    }
    //    std::ofstream file(fileName, ios::out | ios::app);
    //    if (file.is_open()){
    ////        file << "Current Yaw: " << yaw << " Target Yaw: " << targetYaw  << " Vision Angle: " <<  (atan2(y,x) * 180.0 / C_PI)+90.0 << std::endl;
    //        file << "Current VeloX: " << veloX << " Target VeloX: " << flightControlData.x << std::endl;
    //        file << "Current VeloY: " << veloY << " Target VeloY: " << flightControlData.y << std::endl;
    //    }
    //    file.close();
}

void VehicleNode::sendFlightCommands(){
    Vehicle* vehicle = ptr_wrapper_->getVehicle();
    cout << "Start Thread Sending flightControl Commands\n";
    while(true){
        //cout << "flightcntrlThread\n";
        if(!findTargetId){
            //vehicle->control->flightCtrl(flightControlData);
            dji_sdk::JoystickCommand joystickCommand;
            joystickCommand.x   = flightControlData.x;
            joystickCommand.y   = flightControlData.y;
            joystickCommand.z   = flightControlData.z;
            joystickCommand.yaw = flightControlData.yaw;
            ptr_wrapper_->velocityAndYawRateCtrlBody(joystickCommand, 30);

            //usleep(200000); // 5Hz
        }
    }
    cout << "exiting\n";
}

void VehicleNode::stopDetector(){
    Vehicle* vehicle = ptr_wrapper_->getVehicle();
    boost::posix_time::ptime my_time = ros::Time::now().toBoost();
    time_str = boost::posix_time::to_iso_string(my_time);
    fileName = "/home/jetson/catkin_ws/trackCar" + time_str.substr(0, 15) + ".txt";
    std::cout << "Detector Stop Requested!!!!" << std::endl;
    std::cout << "set booleans" << std::endl;
    findTargetId = true;
    startTrackingFlag = false;
    nearestUsed=false;
    videoOut=false;
    std::cout << "clear vectors" << std::endl;
    actives.clear();
    classIds.clear();
    std::cout << "destroy windows" << std::endl;
    cv::destroyAllWindows();
    std::cout << "release authority" << std::endl;
    vehicle->control->releaseCtrlAuthority(2);
    hasAuthority=false;
    tempX=0;
    tempY=0;
    std::cout << "\n\n\n\n\n\n\n\n\n\n---------- RELEASE VIDEO ----------\n\n\n\n\n\n\n\n\n\n";
    videoWriter.release();
    ppmx=-1;
    ppmy=-1;
    heightAboveGround=-1;
    fps=-1;
    bbox = Rect2d(0,0,0,0);
    logFile.close();
}

void VehicleNode::publishFPVCameraImage(CameraRGBImage rgbImg, void* userData)
{
    VehicleNode *node_ptr = (VehicleNode *)userData;
    
    try {
        sensor_msgs::Image img;
        img.height = rgbImg.height;
        img.width = rgbImg.width;
        img.step = rgbImg.width*3;
        img.encoding = "rgb8";
        img.data = rgbImg.rawData;

        img.header.stamp = ros::Time::now();
        img.header.frame_id = "FPV_CAMERA";
        node_ptr->fpv_camera_stream_publisher_.publish(img);
    }catch (std::bad_alloc e){
        cout << "std Exception caught: " << e.what() << endl;
    }

//   try{
//         Net net = VehicleNode::getNetwork();
//         Mat blob;
//         Mat mat(rgbImg.height, rgbImg.width, CV_8UC3, rgbImg.rawData.data(), rgbImg.width*3);
//         cvtColor(mat, mat, COLOR_RGB2BGR);
//         //resize(mat, mat, cvSize(740, 416));
//         // Create a window
//         static const string kWinName = "Deep learning object detection - FPV Camera";
//         //namedWindow(kWinName, WINDOW_AUTOSIZE);
//         //imshow(kWinName, mat);
//         //cv::waitKey(1);
//         // Create a 4D blob from a frame.
//         blobFromImage(mat, blob, 1/255.0, cvSize(480, 480), Scalar(0,0,0), true, false);
//         //cout << "blob size: " << blob.size() << endl;
//         //cout << "#### Got image from:\t" << name << endl;
//         //Sets the input to the network
//         net.setInput(blob);
//         //cout << "#### setInput\t" << name << endl;
//         // Runs the forward pass to get output of the output layers
//         vector<Mat> outs;
//         //static vector<String> outputNames = getOutputsNames(net);
//         net.forward(outs, getOutputsNames(net));
//         //cout << "#### forward\t" << name << endl;
//         // Remove the bounding boxes with low confidence
//         postprocess1(mat, outs, 0);
//         //cout << "#### postprocess\t" << name << endl;
//         // Put efficiency information. The function getPerfProfile returns the overall time for inference(t) and the timings for each of the layers(in layersTimes)
//         vector<double> layersTimes;
//         double freq = getTickFrequency() / 1000;
//         double t = net.getPerfProfile(layersTimes) / freq;
//         string label = format("Inference time for a frame : %.2f ms", t);
//         putText(mat, label, Point(0, 15), FONT_HERSHEY_TRIPLEX, 0.5, Scalar(0, 0, 255));
//         //cout << "#### putText\t" << name << endl;
//         // Write the frame with the detection boxes
//         Mat detectedFrame;
//         mat.convertTo(detectedFrame, CV_8U);
//         //imshow(kWinName, mat);
//         //waitKey(1);
//         dji_sdk::BoundingBoxes boundBoxes;
//         boundBoxes.header = img.header;
//         vector<string> classes = VehicleNode::getClasses();
//         //cout << "detection size: " << classIds.size() << endl;
//         boundBoxes.labels.resize(classIds.size());
//         boundBoxes.confidences.resize(classIds.size());
//         boundBoxes.tops.resize(classIds.size());
//         boundBoxes.lefts.resize(classIds.size());
//         boundBoxes.rights.resize(classIds.size());
//         boundBoxes.bottoms.resize(classIds.size());
//         for (int detections=0; detections < classIds.size(); detections++){
//             boundBoxes.labels[detections] = classes[classIds[detections]];
//             boundBoxes.confidences[detections] = confidences[detections];
//             boundBoxes.tops[detections] = (boxes2[detections].x - boxes2[detections].width/2)/(rgbImg.width * 1.0);
//             boundBoxes.lefts[detections] = (boxes2[detections].y - boxes2[detections].height/2)/(rgbImg.height * 1.0);
//             boundBoxes.rights[detections] = (boxes2[detections].x + boxes2[detections].width/2)/(rgbImg.width * 1.0);
//             boundBoxes.bottoms[detections] = (boxes2[detections].y + boxes2[detections].height/2)/(rgbImg.height * 1.0);
//         }
//         node_ptr->bounding_boxes_publisher_.publish(boundBoxes);
//         //cout << "#### imgShow\t" << name << endl;
//     }catch (cv::Exception e){
//         cout << "CV Exception caught: " << e.what() << endl;
//     }catch (std::bad_alloc e){
//         cout << "std Exception caught: " << e.what() << endl;
//     }
}

void VehicleNode::publishCameraH264(uint8_t* buf, int bufLen, void* userData)
{
  if (userData)
  {
    try{
	    VehicleNode *node_ptr = reinterpret_cast<VehicleNode*>(userData);
	    std::vector<uint8_t> tempRawData(buf, buf+bufLen);
	    sensor_msgs::Image img;
	    img.header.stamp = ros::Time::now();
	    img.data = tempRawData;
	    node_ptr->camera_h264_publisher_.publish(img);

	    H264Decoder decoder;
	    cout << "len of buf: " << tempRawData.size() << endl;
	    decoder.decode(buf, bufLen);
	    Mat mat = decoder.getMat().clone();
	    static const string kWinName = "H264 Stream Test - Main Camera";
	    namedWindow(kWinName, WINDOW_AUTOSIZE);
	    imshow(kWinName, mat);
	    waitKey(1);

    }catch(cv::Exception){
	    cout << "cv exception" << endl;
    }
  } 
  else 
  {
   DERROR("userData is a null value (should be a pointer to VehicleWrapper).");
  }
}

#endif
