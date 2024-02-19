//
// Created by dji on 2020/5/8.
//

#include <dji_sdk/dji_vehicle_node.h>
using namespace dji_sdk;

void VehicleNode::SDKfromMobileDataCallback(Vehicle *vehicle, RecvContainer recvFrame, DJI::OSDK::UserData userData) {
  ((VehicleNode*)userData)->fromMobileDataCallback(recvFrame);
}

void VehicleNode::fromMobileDataCallback(RecvContainer recvFrame) {
  int dataLength = recvFrame.recvInfo.len - OpenProtocol::PackageMin - 2;
  if (dataLength <= 100) {
    DSTATUS( "Received mobile Data of len %d\n", recvFrame.recvInfo.len);
    dji_sdk::MobileData mobile_data;
    mobile_data.data.resize(dataLength);
    float targetCoordinates[dataLength];
    int counter = 0;
    int powerIndx = 0;
    //std::cout << "Received: " <<  recvFrame.recvData.raw_ack_array[0]-48 << " counter: " << counter << " powerIndex: " << powerIndx << std::endl;

    /*GimbalRotationData gimbalRotationData;
    gimbalRotationData.rotationMode = 1;
    gimbalRotationData.pitch = -90;
    gimbalRotationData.roll  = 0;
    gimbalRotationData.yaw   = 0;
    gimbalRotationData.time  = 0;
    bool gimbal_result = ptr_wrapper_->rotateGimbal(static_cast<PayloadIndex>(0), gimbalRotationData);
*/

    for (int i=dataLength-1; i>=0; i--)
    {
	std::cout << "Received: " <<  recvFrame.recvData.raw_ack_array[i]-48 << " counter: " << counter << " powerIndex: " << powerIndx << std::endl;
	if(recvFrame.recvData.raw_ack_array[i] != 44){//if is not delimiter ','
		mobile_data.data[counter] += pow(10,powerIndx)*(recvFrame.recvData.raw_ack_array[i]-48);
		powerIndx++;
	}else{
		targetCoordinates[counter] = mobile_data.data[counter] / 100.0;
		cout << "Decoded Received Number: " << mobile_data.data[counter] / 100.0 << endl;
		counter++;
		powerIndx=0;
	}
    }
    if(powerIndx>0){
		targetCoordinates[counter] = mobile_data.data[counter] / 100.0;
		cout << "Decoded Received Number: " << mobile_data.data[counter] / 100.0 << endl;
    }
    mobile_data.data.resize(counter+1);
    from_mobile_data_publisher_.publish(mobile_data);
    
    if (mobile_data.data[0]==253)
    	stopDetector();
    else if(mobile_data.data[0]==252){
	std::cout << "Shutdown command received" << std::endl;
	system("shutdown -P now");
    }else{
    	//stopDetector();
	std::cout << "TRACK BOX NEAREST TO X: " << targetCoordinates[1] << " Y: " << targetCoordinates[0] << std::endl;
    	//nearestBox(targetCoordinates[1], targetCoordinates[0]);
    }
  }
}

bool VehicleNode::sendToMobileCallback(dji_sdk::SendMobileData::Request& request,
                                       dji_sdk::SendMobileData::Response& response){
  ROS_DEBUG("called sendToMobileCallback");
  response.result = false;
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return true;
  }
  ptr_wrapper_->sendDataToMSDK(&request.data[0], request.data.size());
  response.result = true;
  return true;
}
