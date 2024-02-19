/** @file dji_vehicle_node.hpp
 *  @version 4.0
 *  @date May 2020
 *
 *  @brief main node of osdk ros 4.0.All services and topics are inited here.
 *
 *  @Copyright (c) 2020 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

//INCLUDE
#include <dji_sdk/dji_vehicle_node.h>
#include <dji_sdk/vehicle_wrapper.h>

#ifdef OPEN_CV_INSTALLED
#include <dji_sdk/stereo_utility/m300_stereo_param_tool.hpp>
#endif

#include <unistd.h>
#include <fstream>

#include <vector>
//CODE
using namespace dji_sdk;
#define M300_FRONT_STEREO_PARAM_YAML_NAME "m300_front_stereo_param.yaml"

int dummy_error_handler(int status, char const* func_name, char const* err_msg, char const* file_name, int line, void* userdata)
{
    //Do nothing -- will suppress console output
    return 0;   //Return value is not used
}

VehicleNode::VehicleNode():telemetry_from_fc_(TelemetryType::USE_ROS_BROADCAST),
                           R_FLU2FRD_(tf::Matrix3x3(1,  0,  0, 0, -1,  0, 0,  0, -1)),
                           R_ENU2NED_(tf::Matrix3x3(0,  1,  0, 1,  0,  0, 0,  0, -1)),
                           curr_align_state_(AlignStatus::UNALIGNED)
{
    const char* boardIdFile = "/var/lib/dbus/machine-id";
    ifstream f;
    f.open(boardIdFile, std::ifstream::in);
    std::getline(f, boardId);
    f.close();

    boardIdSubstring = boardId.substr(0, 4);

    publishName = std::string("matrice300_") + std::string(boardIdSubstring);
    std::cout << "\n### Publish Name:\t" << publishName << " ###\n";

  std::cout << "Setting up vehicle_node...\n";
  nh_.param("/vehicle_node_"+boardIdSubstring+"/app_id",        app_id_, 12345);
  nh_.param("/vehicle_node_"+boardIdSubstring+"/enc_key",       enc_key_, std::string("abcde123"));
  nh_.param("/vehicle_node_"+boardIdSubstring+"/acm_name",      device_acm_, std::string("/dev/ttyACM0"));
  nh_.param("/vehicle_node_"+boardIdSubstring+"/serial_name",   device_, std::string("/dev/ttyUSB0"));
  nh_.param("/vehicle_node_"+boardIdSubstring+"/baud_rate",     baud_rate_, 921600);
  nh_.param("/vehicle_node_"+boardIdSubstring+"/app_version",   app_version_, 1);
  nh_.param("/vehicle_node_"+boardIdSubstring+"/drone_version", drone_version_, std::string("M300")); // choose M300 as default
  nh_.param("/vehicle_node_"+boardIdSubstring+"/gravity_const", gravity_const_, 9.801);
  nh_.param("/vehicle_node_"+boardIdSubstring+"/align_time",    align_time_with_FC_, false);
  nh_.param("/vehicle_node_"+boardIdSubstring+"/use_broadcast", user_select_broadcast_, false);
  bool enable_ad = false;
#ifdef ADVANCED_SENSING
  cv::redirectError(dummy_error_handler);
  enable_ad = true;
  /*
  // Load names of classes
  string classesFile = "coco.names";
  ifstream ifs(classesFile.c_str());
  string line;
  while (getline(ifs, line)) classes.push_back(line);

  // Give the configuration and weight files for the model
  String modelConfiguration = "yolov3-tiny.cfg";
  String modelWeights = "yolov3-tiny.weights";

  // Load the network
  net = readNetFromDarknet(modelConfiguration, modelWeights);
  net.setPreferableBackend(DNN_BACKEND_CUDA);
  net.setPreferableTarget(DNN_TARGET_CUDA);

  string classesFile_harpy = "vehicles.names";
 // string classesFile_harpy = "boats.names";
  ifstream ifs_harpy(classesFile_harpy.c_str());
  while (getline(ifs_harpy, line)) classes_harpy.push_back(line);

   // Give the configuration and weight files for the model
   String modelConfiguration_harpy = "DroNet_uav_mix.cfg";
   String modelWeights_harpy = "DroNet_uav_mix_best.weights";
   //String modelConfiguration_harpy = "boats_tyv4.cfg";
   //String modelWeights_harpy = "boats_tyv4_best.weights";

   // Load the network
   net_harpy = readNet(modelWeights_harpy, modelConfiguration_harpy);
   net_harpy.setPreferableBackend(DNN_BACKEND_CUDA);
   net_harpy.setPreferableTarget(DNN_TARGET_CUDA);

   int n = 4; // Number of statesgetClassesHarpy
   int m = 2; // Number of measurements

   double dt = 0.1; //1.0/30; // Time step

   Eigen::MatrixXd A(n, n); // System dynamics matrix
   Eigen::MatrixXd C(m, n); // Output matrix
   Eigen::MatrixXd Q(n, n); // Process noise covariance
   Eigen::MatrixXd R(m, m); // Measurement noise covariance
   Eigen::MatrixXd P(n, n); // Estimate error covariance

   // Discrete LTI projectile motion, measuring position only
   A << 1, 0, dt, 0, 0, 1, 0, dt, 0, 0, 1, 0, 0, 0, 0, 1;
   C << 1, 0, 0, 0, 0, 1, 0, 0;
   //C << 0, 0, 1, 0, 0, 0, 0, 1;

   float q, r;
   ifstream kalmanParamFile;
   kalmanParamFile.open("/home/jetson/catkin_ws/kalmanParam.txt");
   if(!kalmanParamFile){
       q=0.01;
       r=0.01;
   }else
       kalmanParamFile >> q >> r;

   // Reasonable covariance matrices
   Q << q,0,0,0,  0,q,0,0,   0,0,q,0,   0,0,0,q; //Process Noise
   R << r, 0, 0, r;						 //Measurement Noise
   //Q << 0.001,0,0,0,  0,0.001,0,0,   0,0,0.01,0,   0,0,0,0.01;
   //R << .001, 0, 0, .001;
   P << 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;

   std::cout << "A: \n" << A << std::endl;
   std::cout << "C: \n" << C << std::endl;
   std::cout << "Q: \n" << Q << std::endl;
   std::cout << "R: \n" << R << std::endl;
   std::cout << "P: \n" << P << std::endl;

   // Construct the filter
   kf = KalmanFilterCustom(dt, A, C, Q, R, P);

   // Construct the OpenCV tracker
   int type;
   ifstream trackerFile;
   trackerFile.open("/home/jetson/catkin_ws/trackerType.txt");
   if(!trackerFile){ADVANCED_SENSING
	type=2;
   }else
       trackerFile >> type;
   trackerFile.close();
   
   // List of tracker types in OpenCV 3.4.1
   string trackerTypes[8] = {"BOOSTING", "MIL", "KCF", "TLD","MEDIANFLOW", "GOTURN", "MOSSE", "CSRT"};
   string trackerType = trackerTypes[type];

   if (trackerType == "BOOSTING")
           tracker = TrackerBoosting::create();
   else if (trackerType == "MIL")
           tracker = TrackerMIL::create();
   else if (trackerType == "KCF")
           tracker = TrackerKCF::create();
   else if (trackerType == "TLD")
           tracker = TrackerTLD::create();
   else if (trackerType == "MEDIANFLOW")
           tracker = TrackerMedianFlow::create();
   else if (trackerType == "GOTURN")
           tracker = TrackerGOTURN::create();
   else if (trackerType == "MOSSE")
           tracker = TrackerMOSSE::create();
   else if (trackerType == "CSRT")
           tracker = TrackerCSRT::create();
   std::cout << "TRACKER TYPE: " << trackerType << std::endl; 

   int max;
   float p1, i1, d1, p2, i2, d2;
   ifstream pidFile;
   pidFile.open("/home/jetson/catkin_ws/pidX.txt");
   if(!pidFile){
       max = 20;
       p1=0.08;
       i1=0.001;
       d1=0.2;
       p2=0.08;
       i2=0.001;
       d2=0.2;
   }else
       pidFile >> max >> p1 >> i1 >> d1 >> p2 >> i2 >> d2;
   pidFile.close();

   std::cout << "PID X VALUES: pos: " << p1 << ", " << i1 << ", " << d1 << ", vel: " << p2 << ", " << i2 << ", " << d2 << ", Max Speed: ±" << max << std::endl; 

   controlPIDx = PID(dt, max, -max, p1, i1, d1, p2, i2, d2);


   pidFile.open("/home/jetson/catkin_ws/pidY.txt");
   if(!pidFile){
       max = 20;
       p1=0.04;
       i1=0.001;
       d1=0.25;
       p2=0.04;
       i2=0.001;
       d2=0.25;
   }else
       pidFile >> max >> p1 >> i1 >> d1 >> p2 >> i2 >> d2;
   pidFile.close();

   std::cout << "PID Y VALUES: pos: " << p1 << ", " << i1 << ", " << d1 << ", vel: " << p2 << ", " << i2 << ", " << d2 << ", Max Speed: ±" << max << std::endl; 


   controlPIDy = PID(dt, max, -max, p1, i1, d1, p2, i2, d2);
 */
#else
  enable_ad = false;
#endif
  ptr_wrapper_ = new VehicleWrapper(app_id_, enc_key_, device_acm_, device_, baud_rate_, enable_ad);

  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules inited failed");
    ros::shutdown();
  }
  ROS_INFO_STREAM("VehicleNode Start");

  if (NULL != ptr_wrapper_->getVehicle()->subscribe && (!user_select_broadcast_))
  {
    telemetry_from_fc_ = TelemetryType::USE_ROS_SUBSCRIBE;
  }

  initGimbalModule();
  initCameraModule();
  initService();
  initTopic();


  //start camera stream automatically after initilization

  PayloadIndexType index = static_cast<PayloadIndexType>(0);
  ptr_wrapper_->getVehicle()->cameraManager->setModeSync(index, CameraModule::WorkMode::RECORD_VIDEO, 3);
  ptr_wrapper_->startMainCameraStream(&publishMainCameraImage, this);
  //ptr_wrapper_->startMainCameraStream();
  streaming = true;
}

VehicleNode::~VehicleNode()
{
  if(!ptr_wrapper_->isM100() && telemetry_from_fc_ == TelemetryType::USE_ROS_SUBSCRIBE)
  {
    cleanUpSubscribeFromFC();
  }
  else if(telemetry_from_fc_ == TelemetryType::USE_ROS_BROADCAST)
  {
    int pkgIndex = static_cast<int>(SubscribePackgeIndex::BROADCAST_BUT_NEED_SUBSCRIBE);
    int timeout = 1;
    ptr_wrapper_->teardownSubscription(pkgIndex, timeout);
  }
}

std::string get_current_dir(){
    char buff[100];
    getcwd( buff, 100);
    string current_working_dir(buff);
    return current_working_dir;
}

bool VehicleNode::initGimbalModule()
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  /*! init gimbal modules for gimbalManager */
  ErrorCode::ErrorCodeType ret;
  /*! main gimbal init */
  ret = ptr_wrapper_->initGimbalModule(dji_sdk::PayloadIndex::PAYLOAD_INDEX_0, "main_gimbal");
  if (ret != ErrorCode::SysCommonErr::Success)
  {
    std::cout << "Init Camera modules main_gimbal failed."<< std::endl;
    ErrorCode::printErrorCodeMsg(ret);
    return false;
  }
  /*! vice gimbal init */
  ret = ptr_wrapper_->initGimbalModule(dji_sdk::PayloadIndex::PAYLOAD_INDEX_1, "vice_gimbal");
  if (ret != ErrorCode::SysCommonErr::Success)
  {
    std::cout << "Init Camera modules vice_gimbal failed." << std::endl;
    ErrorCode::printErrorCodeMsg(ret);
    return false;
  }
  /*! top gimbal init */
  if (ptr_wrapper_->isM300())
  {
    ret = ptr_wrapper_->initGimbalModule(dji_sdk::PayloadIndex::PAYLOAD_INDEX_2, "top_gimbal");
    if (ret != ErrorCode::SysCommonErr::Success) {
      std::cout << "Init Camera modules top_gimbal failed." << std::endl;
      ErrorCode::printErrorCodeMsg(ret);
      return false;
    }
  }

  return true;
}

bool VehicleNode::initCameraModule()
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  /*! init camera modules for cameraManager */
  /*! main camera init */
  ErrorCode::ErrorCodeType ret = ptr_wrapper_->initCameraModule(
      dji_sdk::PayloadIndex::PAYLOAD_INDEX_0, "main_camera");
  if (ret != ErrorCode::SysCommonErr::Success) {
    DERROR("Init Camera modules main_camera failed.");
    ErrorCode::printErrorCodeMsg(ret);
    return false;
  }
  /*! vice camera init */
  ret = ptr_wrapper_->initCameraModule(dji_sdk::PayloadIndex::PAYLOAD_INDEX_1, "vice_camera");
  if (ret != ErrorCode::SysCommonErr::Success) {
    DERROR("Init Camera modules vice_camera failed.");
    ErrorCode::printErrorCodeMsg(ret);
    return false;
  }
  /*! top camera init for M300 */
  if (ptr_wrapper_->isM300()) {
    ret = ptr_wrapper_->initCameraModule(dji_sdk::PayloadIndex::PAYLOAD_INDEX_2, "top_camera");
    if (ret != ErrorCode::SysCommonErr::Success)
    {
      DERROR("Init Camera modules top_camera failed.");
      ErrorCode::printErrorCodeMsg(ret);
      return false;
    }
  }

  return true;
}

void VehicleNode::initService()
{
  ROS_INFO_STREAM("Topic startup!");
  /*! @brief
   *  general server
   *  @platforms M210V2, M300
   */

  /*const char* boardIdFile = "/var/lib/dbus/machine-id";
  std::string boardId;
  ifstream f;
  f.open(boardIdFile, std::ifstream::in);
  std::getline(f, boardId);
  f.close();

  boardIdSubstring = boardId.substr(0, 4);

  publishName = std::string("matrice300_") + std::string(boardIdSubstring);*/
  std::cout << "\n### Service Publish Name:\t" << publishName << " ###\n";

  get_drone_type_server_ = nh_.advertiseService(publishName+"/get_drone_type", &VehicleNode::getDroneTypeCallback, this);

  /*! @brief
   *  flight control server
   *  @platforms M210V2, M300
   */
  task_control_server_ = nh_.advertiseService(publishName+"/flight_task_control", &VehicleNode::taskCtrlCallback, this);
  joystick_action_server_ = nh_.advertiseService(publishName+"/joystick_action", &VehicleNode::JoystickActionCallback, this);
  set_joystick_mode_server_ = nh_.advertiseService(publishName+"/set_joystick_mode", &VehicleNode::setJoystickModeCallback, this);
  set_home_altitude_server_ = nh_.advertiseService(publishName+"/set_go_home_altitude", &VehicleNode::setGoHomeAltitudeCallback,this);
  get_home_altitude_server_ = nh_.advertiseService(publishName+"/get_go_home_altitude", &VehicleNode::getGoHomeAltitudeCallback,this);
  set_current_aircraft_point_as_home_server_ = nh_.advertiseService(publishName+"/set_current_aircraft_point_as_home",
                                               &VehicleNode::setCurrentAircraftLocAsHomeCallback,this);
  set_home_point_server_ = nh_.advertiseService(publishName+"/set_home_point", &VehicleNode::setHomePointCallback, this);
  set_local_pos_reference_server_ = nh_.advertiseService(publishName+"/set_local_pos_reference", &VehicleNode::setLocalPosRefCallback,this);
  set_horizon_avoid_enable_server_ = nh_.advertiseService(publishName+"/set_horizon_avoid_enable", &VehicleNode::setHorizonAvoidCallback,this);
  set_upwards_avoid_enable_server_ = nh_.advertiseService(publishName+"/set_upwards_avoid_enable", &VehicleNode::setUpwardsAvoidCallback, this);
  get_avoid_enable_status_server_ = nh_.advertiseService(publishName+"/get_avoid_enable_status", &VehicleNode::getAvoidEnableStatusCallback, this);
  obtain_releae_control_authority_server_ = nh_.advertiseService(publishName+"/obtain_release_control_authority",
                                            &VehicleNode::obtainReleaseControlAuthorityCallback, this);
  kill_switch_server_ = nh_.advertiseService(publishName+"/kill_switch", &VehicleNode::killSwitchCallback, this);
  emergency_brake_action_server_ = nh_.advertiseService(publishName+"/emergency_brake", &VehicleNode::emergencyBrakeCallback, this);
  /*! @brief
   *  gimbal control server
   *  @platforms M210V2, M300
   */
  gimbal_control_server_ = nh_.advertiseService(publishName+"/gimbal_task_control", &VehicleNode::gimbalCtrlCallback, this);

  /*! @brief
   *  camera control server
   *  @platforms M210V2, M300
   */
  camera_control_set_EV_server_ = nh_.advertiseService(publishName+"/camera_task_set_EV", &VehicleNode::cameraSetEVCallback, this);
  camera_control_set_shutter_speed_server_ = nh_.advertiseService(publishName+"/camera_task_set_shutter_speed", &VehicleNode::cameraSetShutterSpeedCallback, this);
  camera_control_set_aperture_server_ = nh_.advertiseService(publishName+"/camera_task_set_aperture", &VehicleNode::cameraSetApertureCallback, this);
  camera_control_set_ISO_server_ = nh_.advertiseService(publishName+"/camera_task_set_ISO", &VehicleNode::cameraSetISOCallback, this);
  camera_control_set_focus_point_server_ = nh_.advertiseService(publishName+"/camera_task_set_focus_point", &VehicleNode::cameraSetFocusPointCallback, this);
  camera_control_set_tap_zoom_point_server_ = nh_.advertiseService(publishName+"/camera_task_tap_zoom_point", &VehicleNode::cameraSetTapZoomPointCallback, this);
  camera_control_set_zoom_para_server_ = nh_.advertiseService(publishName+"/camera_task_set_zoom_para", &VehicleNode::cameraSetZoomParaCallback, this);
  camera_control_zoom_ctrl_server_ = nh_.advertiseService(publishName+"/camera_task_zoom_ctrl", &VehicleNode::cameraZoomCtrlCallback, this);
  camera_control_start_shoot_single_photo_server_ = nh_.advertiseService(publishName+"/camera_start_shoot_single_photo", &VehicleNode::cameraStartShootSinglePhotoCallback, this);
  camera_control_start_shoot_AEB_photo_server_ = nh_.advertiseService(publishName+"/camera_start_shoot_aeb_photo", &VehicleNode::cameraStartShootAEBPhotoCallback, this);
  camera_control_start_shoot_burst_photo_server_ = nh_.advertiseService(publishName+"/camera_start_shoot_burst_photo", &VehicleNode::cameraStartShootBurstPhotoCallback, this);
  camera_control_start_shoot_interval_photo_server_ = nh_.advertiseService(publishName+"/camera_start_shoot_interval_photo", &VehicleNode::cameraStartShootIntervalPhotoCallback, this);
  camera_control_stop_shoot_photo_server_ = nh_.advertiseService(publishName+"/camera_stop_shoot_photo", &VehicleNode::cameraStopShootPhotoCallback, this);
  camera_control_record_video_action_server_ = nh_.advertiseService(publishName+"/camera_record_video_action", &VehicleNode::cameraRecordVideoActionCallback, this);

  /* @brief
   * get whole battery info server
   * @platforms M210V2
   */
  get_whole_battery_info_server_ = nh_.advertiseService(publishName+"/get_whole_battery_info", &VehicleNode::getWholeBatteryInfoCallback, this);
  get_single_battery_dynamic_info_server_ = nh_.advertiseService(publishName+"/get_single_battery_dynamic_info", &VehicleNode::getSingleBatteryDynamicInfoCallback, this);
  
  /*! @brief
   *  mfio control server
   *  @platforms M300
   */
  get_hms_data_server_ = nh_.advertiseService(publishName+"/get_hms_data", &VehicleNode::getHMSDataCallback, this);
  /*! @brief
   *  mfio control server
   *  @platforms null
   */
  mfio_control_server_ = nh_.advertiseService(publishName+"/mfio_control", &VehicleNode::mfioCtrlCallback, this);

  /*! @brief
   *  mobile device server
   *  @platforms M210V2, M300
   */
  send_data_to_mobile_device_server_ = nh_.advertiseService(publishName+"/send_data_to_mobile_device", &VehicleNode::sendToMobileCallback, this);
  send_data_to_mobile_device_client_ = nh_.serviceClient<dji_sdk::SendMobileData>(publishName+"/send_data_to_mobile_device");

  /*! @brief
   *  payload device server
   *  @platforms M210V2, M300
   */
  send_data_to_payload_device_server_ = nh_.advertiseService(publishName+"/send_data_to_payload_device_server", &VehicleNode::sendToPayloadCallback, this);

  /*! @brief
   *  advanced sensing server
   *  @platforms M210V2, M300
   */
#ifdef ADVANCED_SENSING
  setup_camera_stream_server_ = nh_.advertiseService(publishName+"/setup_camera_stream", &VehicleNode::setupCameraStreamCallback, this);
  setup_camera_h264_server_ = nh_.advertiseService(publishName+"/setup_camera_h264", &VehicleNode::setupCameraH264Callback, this);
  subscribe_stereo_240p_server_  = nh_.advertiseService(publishName+"/stereo_240p_subscription",   &VehicleNode::stereo240pSubscriptionCallback, this);
  subscribe_stereo_depth_server_ = nh_.advertiseService(publishName+"/stereo_depth_subscription",  &VehicleNode::stereoDepthSubscriptionCallback,this);
  subscribe_stereo_vga_server_   = nh_.advertiseService(publishName+"/stereo_vga_subscription",    &VehicleNode::stereoVGASubscriptionCallback,  this);
#ifdef OPEN_CV_INSTALLED
  /*! @brief
   *  get m300 stereo params server
   *  @platforms M300
   */
  //get_m300_stereo_params_server_ = nh_.advertiseService(publishName+"/get_m300_stereo_params", &VehicleNode::getM300StereoParamsCallback, this);
#endif
#endif
  /*! @brief
   *  waypointV1.0 server
   *  @platforms M210
   */
  waypoint_upload_server_    = nh_.advertiseService(publishName+"/mission_waypoint_upload",        &VehicleNode::missionWpUploadCallback,        this);
  //waypoint_action_server_    = nh_.advertiseService(publishName+"/mission_waypoint_action",        &VehicleNode::missionWpActionCallback,        this);
  waypoint_getInfo_server_   = nh_.advertiseService(publishName+"/mission_waypoint_getInfo",       &VehicleNode::missionWpGetInfoCallback,       this);
  waypoint_getSpeed_server_  = nh_.advertiseService(publishName+"/mission_waypoint_getSpeed",      &VehicleNode::missionWpGetSpeedCallback,      this);
  waypoint_setSpeed_server_  = nh_.advertiseService(publishName+"/mission_waypoint_setSpeed",      &VehicleNode::missionWpSetSpeedCallback,      this);

  /*! @brief
   *  hotpoint server
   *  @platforms M210, M300
   */
  hotpoint_upload_server_    = nh_.advertiseService(publishName+"/mission_hotpoint_upload",        &VehicleNode::missionHpUploadCallback,        this);
  hotpoint_action_server_    = nh_.advertiseService(publishName+"/mission_hotpoint_action",        &VehicleNode::missionHpActionCallback,        this);
  hotpoint_getInfo_server_   = nh_.advertiseService(publishName+"/mission_hotpoint_getInfo",       &VehicleNode::missionHpGetInfoCallback,       this);
  hotpoint_setSpeed_server_  = nh_.advertiseService(publishName+"/mission_hotpoint_updateYawRate", &VehicleNode::missionHpUpdateYawRateCallback, this);
  hotpoint_resetYaw_server_  = nh_.advertiseService(publishName+"/mission_hotpoint_resetYaw",      &VehicleNode::missionHpResetYawCallback,      this);
  hotpoint_setRadius_server_ = nh_.advertiseService(publishName+"/mission_hotpoint_updateRadius",  &VehicleNode::missionHpUpdateRadiusCallback,  this);
  mission_status_server_     = nh_.advertiseService(publishName+"/mission_status",                 &VehicleNode::missionStatusCallback,          this);

  /*! @brief
   *  waypoint2.0 server
   *  @platforms M300
   */
  waypointV2_init_setting_server_     = nh_.advertiseService(publishName+"/waypointV2_initSetting",    &VehicleNode::waypointV2InitSettingCallback, this);
  waypointV2_upload_mission_server_   = nh_.advertiseService(publishName+"/waypointV2_uploadMission", &VehicleNode::waypointV2UploadMissionCallback, this);
  waypointV2_download_mission_server_ = nh_.advertiseService(publishName+"/waypointV2_downloadMission", &VehicleNode::waypointV2DownloadMissionCallback, this);
  waypointV2_upload_action_server_    = nh_.advertiseService(publishName+"/waypointV2_uploadAction", &VehicleNode::waypointV2UploadActionCallback, this);
  waypointV2_start_mission_server_    = nh_.advertiseService(publishName+"/waypointV2_startMission", &VehicleNode::waypointV2StartMissionCallback, this);
  waypointV2_stop_mission_server_     = nh_.advertiseService(publishName+"/waypointV2_stopMission", &VehicleNode::waypointV2StopMissionCallback, this);
  waypointV2_pause_mission_server_    = nh_.advertiseService(publishName+"/waypointV2_pauseMission", &VehicleNode::waypointV2PauseMissionCallback, this);
  waypointV2_resume_mission_server_   = nh_.advertiseService(publishName+"/waypointV2_resumeMission", &VehicleNode::waypointV2ResumeMissionCallback, this);
  waypointV2_generate_actions_server_ = nh_.advertiseService(publishName+"/waypointV2_generateActions", &VehicleNode::waypointV2GenerateActionsCallback, this);
  waypointv2_set_global_cruisespeed_server_ = nh_.advertiseService(publishName+"/waypointV2_setGlobalCruisespeed", &VehicleNode::waypointV2SetGlobalCruisespeedCallback, this);
  waypointv2_get_global_cruisespeed_server_ = nh_.advertiseService(publishName+"/waypointV2_getGlobalCruisespeed", &VehicleNode::waypointV2GetGlobalCruisespeedCallback, this);
  waypointv2_subscribe_mission_event_server_ = nh_.advertiseService(publishName+"/waypointV2_subscribeMissionEvent", &VehicleNode::waypointV2SubscribeMissionEventCallback, this);
  waypointv2_subscribe_mission_state_server_ = nh_.advertiseService(publishName+"/waypointV2_subscribeMissionState", &VehicleNode::waypointV2SubscribeMissionStateCallback, this);

  ROS_INFO_STREAM("Services startup!");
}

bool VehicleNode::initTopic()
{

  mission_waypoint_upload_report_publisher_ = nh_.advertise<std_msgs::String>(publishName+"/mission_waypoint_report", 10);

  attitude_publisher_ = nh_.advertise<geometry_msgs::QuaternionStamped>(publishName+"/attitude", 10);
/* @brief Provides various data about the battery
 * @note Most of these details need a DJI Intelligent battery to work correctly
 * (this is usually not the case with A3/N3 based setups)
 * @details Please be aware that some of the data elements in this topic may not be able to update
 * at high rates due to the limitations of the sensing for that data. e.g. current can only update @ 1 Hz.
 * @platforms M210,M300
 * @units
 * |voltage           | mV |
 * |current           | mA |
 * @datastruct \ref Battery
 */
  battery_state_publisher_ = nh_.advertise<sensor_msgs::BatteryState>(publishName+"/battery_state",10);
  /*!
   * - Fused attitude (duplicated from attitude topic)
   * - Raw linear acceleration (body frame: FLU, m/s^2)
   *       Z value is +9.8 when placed on level ground statically
   * - Raw angular velocity (body frame: FLU, rad/s)
   */
  imu_publisher_ = nh_.advertise<sensor_msgs::Imu>(publishName+"/imu", 10);
  // Refer to dji_sdk.h for different enums for M100 and A3/N3
  flight_status_publisher_ = nh_.advertise<std_msgs::UInt8>(publishName+"/flight_status", 10);
  /*!
   * gps_health needs to be greater than 3 for gps_position and velocity topics
   * to be trusted
   */
  gps_health_publisher_ = nh_.advertise<std_msgs::UInt8>(publishName+"/gps_health", 10);

  /*!
   * NavSatFix specs:
   *   Latitude [degrees]. Positive is north of equator; negative is south.
   *   Longitude [degrees]. Positive is east of prime meridian; negative is
   * west.
   *   Altitude [m]. Positive is above the WGS 84 ellipsoid
   */
  gps_position_publisher_ = nh_.advertise<sensor_msgs::NavSatFix>(publishName+"/gps_position", 10);


  telemetry_tanslator_publisher_ = nh_.advertise<dji_sdk::telemetry2>(publishName+"/telemetry2", 10);

  wind_data_publisher_ = nh_.advertise<dji_sdk::WindData>(publishName+"/wind_data", 10);

  /*!
   *   x [m]. Positive along navigation frame x axis
   *   y [m]. Positive along navigation frame y axis
   *   z [m]. Positive is down
   *   For details about navigation frame, please see telemetry documentation in API reference
  */
  vo_position_publisher_ = nh_.advertise<dji_sdk::VOPosition>(publishName+"/vo_position", 10);
  /*!
   * Height above home altitude. It is valid only after drone
   * is armed.
   */
  height_publisher_ = nh_.advertise<std_msgs::Float32>(publishName+"/height_above_takeoff", 10);
  velocity_publisher_ = nh_.advertise<geometry_msgs::Vector3Stamped>(publishName+"/velocity", 10);
  from_mobile_data_publisher_ = nh_.advertise<dji_sdk::MobileData>(publishName+"/from_mobile_data", 10);
  from_payload_data_publisher_ = nh_.advertise<dji_sdk::PayloadData>(publishName+"/from_payload_data", 10);
  // TODO: documentation and proper frame id
  gimbal_angle_publisher_ = nh_.advertise<geometry_msgs::Vector3Stamped>(publishName+"/gimbal_angle", 10);
  rc_publisher_ = nh_.advertise<sensor_msgs::Joy>(publishName+"/rc", 10);

  local_position_publisher_ = nh_.advertise<geometry_msgs::PointStamped>(publishName+"/local_position", 10);
  local_frame_ref_publisher_ = nh_.advertise<sensor_msgs::NavSatFix>(publishName+"/local_frame_ref", 10, true);
  time_sync_nmea_publisher_ = nh_.advertise<nmea_msgs::Sentence>(publishName+"/time_sync_nmea_msg", 10);
  time_sync_gps_utc_publisher_ = nh_.advertise<dji_sdk::GPSUTC>(publishName+"/time_sync_gps_utc", 10);
  time_sync_fc_utc_publisher_ = nh_.advertise<dji_sdk::FCTimeInUTC>(publishName+"/time_sync_fc_time_utc", 10);
  time_sync_pps_source_publisher_ = nh_.advertise<std_msgs::String>(publishName+"/time_sync_pps_source", 10);

  #ifdef ADVANCED_SENSING
  main_camera_stream_publisher_ = nh_.advertise<sensor_msgs::Image>(publishName+"/main_camera_images", 1);
  main_camera_stream_resolution_publisher_ = nh_.advertise<dji_sdk::Resolution>(publishName+"/main_camera_stream_resolution", 1);
  main_camera_photo_resolution_publisher_ = nh_.advertise<dji_sdk::Resolution>(publishName+"/main_camera_photo_resolution", 1);

  main_camera_stream_270p30fps_publisher_ = nh_.advertise<sensor_msgs::Image>(publishName+"/main_camera/270p30fps/stream", 1, true);
  main_camera_stream_270p30fps_resolution_publisher_ = nh_.advertise<dji_sdk::Resolution>(publishName+"/main_camera/270p30fps/stream_resolution", 1);
  //main_camera_photo_live_resolution_publisher_ = nh_.advertise<dji_sdk::Resolution>(publishName+"/main_camera/main_camera_photo_live_resolution", 1);

  main_camera_sd_contents_publisher_ = nh_.advertise<dji_sdk::ComponentList>(publishName+"/main_camera_sd_content", 10);

  main_camera_parameters_publisher_ = nh_.advertise<std_msgs::Float32>(publishName+"/main_camera_parameters", 10);


  fpv_camera_stream_publisher_ = nh_.advertise<sensor_msgs::Image>(publishName+"/fpv_camera_images", 10);
  camera_h264_publisher_ = nh_.advertise<sensor_msgs::Image>(publishName+"/camera_h264_stream", 10);
  stereo_240p_front_left_publisher_ = nh_.advertise<sensor_msgs::Image>(publishName+"/stereo_240p_front_left_images", 10);
  stereo_240p_front_right_publisher_ = nh_.advertise<sensor_msgs::Image>(publishName+"/stereo_240p_front_right_images", 10);
  stereo_240p_down_front_publisher_ = nh_.advertise<sensor_msgs::Image>(publishName+"/stereo_240p_down_front_images", 10);
  stereo_240p_down_back_publisher_ = nh_.advertise<sensor_msgs::Image>(publishName+"/stereo_240p_down_back_images", 10);
  stereo_240p_front_depth_publisher_ = nh_.advertise<sensor_msgs::Image>(publishName+"/stereo_240p_front_depth_images", 10);
  stereo_vga_front_left_publisher_ = nh_.advertise<sensor_msgs::Image>(publishName+"/stereo_vga_front_left_images", 10);
  stereo_vga_front_right_publisher_ = nh_.advertise<sensor_msgs::Image>(publishName+"/stereo_vga_front_right_images", 10);

  stereo_depth_publisher_ = nh_.advertise<sensor_msgs::Image>(publishName+"/stereo_depth_images", 10);

  bounding_boxes_publisher_ = nh_.advertise<dji_sdk::BoundingBoxes>(publishName+"/bounding_boxes", 10);

  track_stop_server_ = nh_.advertiseService(publishName+"/tracker_service",            &VehicleNode::stopKalmanTrackCallback,        this);


  camera_list_publisher_ = nh_.advertise<dji_sdk::ComponentList>(publishName+"/CameraList", 10);


  //track_car_subscriber_ = nh_.subscribe<std_msgs::Float32MultiArray>(publishName+"/track_car", 1, &VehicleNode::trackCarCallback, this);
  //controlThread = std::thread(&VehicleNode::sendFlightCommands, this); 
  //publishMainCameraThread = std::thread(&VehicleNode::publishPullMainCameraImage, this);
  //controlThread.detach();
  #endif

  waypointV2_mission_state_publisher_ = nh_.advertise<dji_sdk::WaypointV2MissionStatePush>(publishName+"/waypointV2_mission_state", 10);
  waypointV2_mission_event_publisher_ = nh_.advertise<dji_sdk::WaypointV2MissionEventPush>(publishName+"/waypointV2_mission_event", 10);

  scan_area_response_subscriber = nh_.subscribe<dji_sdk::MissionWaypointTask>(
              publishName+"/ScanAreaResponse", 2, &VehicleNode::scanAreaResponseCallback, this);

  esc_data_publisher_ = nh_.advertise<dji_sdk::EscData>(publishName+"/ESC_data", 10);



  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  Vehicle* vehicle = ptr_wrapper_->getVehicle();
  if (true)
  {
    ACK::ErrorCode broadcast_set_freq_ack;
    ROS_INFO("Use legacy data broadcast to get telemetry data!");

    uint8_t defaultFreq[16];

    if (ptr_wrapper_->isM100()) {
      ptr_wrapper_->setUpM100DefaultFreq(defaultFreq);
    } else {
      ptr_wrapper_->setUpA3N3DefaultFreq(defaultFreq);
    }
    broadcast_set_freq_ack = ptr_wrapper_->setBroadcastFreq(defaultFreq, WAIT_TIMEOUT);

    if (ACK::getError(broadcast_set_freq_ack)) {
      ACK::getErrorCodeMessage(broadcast_set_freq_ack, __func__);
      return false;
    }
    // register a callback function whenever a broadcast data is in
    ptr_wrapper_->setUserBroadcastCallback(&VehicleNode::SDKBroadcastCallback, this);

    /*! some data still need to be subscribed*/
    int pkgIndex = static_cast<int>(SubscribePackgeIndex::BROADCAST_BUT_NEED_SUBSCRIBE);
    int freq = 50;
    int timeout = 1;
    std::vector<Telemetry::TopicName> topicList50Hz;

    topicList50Hz.push_back(Telemetry::TOPIC_STATUS_FLIGHT);
    topicList50Hz.push_back(Telemetry::TOPIC_STATUS_DISPLAYMODE);
    topicList50Hz.push_back(Telemetry::TOPIC_VELOCITY);
    topicList50Hz.push_back(Telemetry::TOPIC_GPS_FUSED);
    topicList50Hz.push_back(Telemetry::TOPIC_QUATERNION);

    if (ptr_wrapper_->isM300())
    {
      topicList50Hz.push_back(Telemetry::TOPIC_THREE_GIMBAL_DATA);
      std::cout << "Pushing TOPIC_THREE_GIMBAL_DATA 1\n";
    }
    else
    {
      topicList50Hz.push_back(Telemetry::TOPIC_DUAL_GIMBAL_DATA);

    }
    int topicSize = topicList50Hz.size();;
    ptr_wrapper_->setUpSubscription(pkgIndex, freq, topicList50Hz.data(), topicSize, timeout);
  }
  else if (telemetry_from_fc_ == TelemetryType::USE_ROS_SUBSCRIBE)
  {
    ROS_INFO("Use data subscription to get telemetry data!");
    if(!align_time_with_FC_)
    {
      ROS_INFO("align_time_with_FC set to false. We will use ros time to time stamp messages!");
    }
    else
    {
      ROS_INFO("align_time_with_FC set to true. We will time stamp messages based on flight controller time!");
    }

    // Extra topics that is only available from subscription

    // Details can be found in DisplayMode enum in common_type.h
    angularRate_publisher_ = nh_.advertise<geometry_msgs::Vector3Stamped>(publishName+"/angular_velocity_fused", 10);
    acceleration_publisher_ = nh_.advertise<geometry_msgs::Vector3Stamped>(publishName+"/acceleration_ground_fused", 10);
    displaymode_publisher_ = nh_.advertise<std_msgs::UInt8>(publishName+"/display_mode", 10);
    trigger_publisher_ = nh_.advertise<sensor_msgs::TimeReference>(publishName+"/trigger_time", 10);

    if (!initDataSubscribeFromFC())
    {
      return false;
    }
  }
  ptr_wrapper_->setFromMSDKCallback(&VehicleNode::SDKfromMobileDataCallback, this);
  if (vehicle->payloadDevice)
  {
    ptr_wrapper_->setFromPSDKCallback(&VehicleNode::SDKfromPayloadDataCallback, this);
  }

  if (vehicle->hardSync)
  {
    ptr_wrapper_->subscribeNMEAMsgs(&VehicleNode::NMEACallback, this);
    ptr_wrapper_->subscribeUTCTime(&VehicleNode::GPSUTCTimeCallback, this);
    ptr_wrapper_->subscribeFCTimeInUTCRef(&VehicleNode::FCTimeInUTCCallback, this);
    ptr_wrapper_->subscribePPSSource(&VehicleNode::PPSSourceCallback, this);
  }
  return true;
}

bool VehicleNode::initDataSubscribeFromFC()
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  ACK::ErrorCode ack = ptr_wrapper_->verify(WAIT_TIMEOUT);
  if (ACK::getError(ack))
  {
    return false;
  }

  std::vector<Telemetry::TopicName> topicList1hz;
  topicList1hz.push_back(Telemetry::TOPIC_ALTITUDE_OF_HOMEPOINT);
  int nTopic1hz    = topicList1hz.size();
  if (ptr_wrapper_->initPackageFromTopicList(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_1HZ), nTopic1hz,
                                             topicList1hz.data(), 1, 1))
  {
    ack = ptr_wrapper_->startPackage(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_1HZ), WAIT_TIMEOUT);
    if (ACK::getError(ack))
    {
      ptr_wrapper_->removePackage(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_1HZ), WAIT_TIMEOUT);
      ROS_ERROR("Failed to start 1hz package");
      return false;
    }
    else
    {
      ptr_wrapper_->registerUserPackageUnpackCallback(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_1HZ), VehicleNode::publish1HzData, (UserData) this);
    }
  }

  std::vector<Telemetry::TopicName> topicList100Hz;
  topicList100Hz.push_back(Telemetry::TOPIC_QUATERNION);
  topicList100Hz.push_back(Telemetry::TOPIC_ACCELERATION_GROUND);
  topicList100Hz.push_back(Telemetry::TOPIC_ANGULAR_RATE_FUSIONED);

  int nTopic100Hz    = topicList100Hz.size();
  if (ptr_wrapper_->initPackageFromTopicList(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_100HZ), nTopic100Hz,
                                             topicList100Hz.data(), 1, 100))
  {
    ack = ptr_wrapper_->startPackage(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_100HZ), WAIT_TIMEOUT);
    if (ACK::getError(ack))
    {
      ptr_wrapper_->removePackage(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_100HZ), WAIT_TIMEOUT);
      ROS_ERROR("Failed to start 100Hz package");
      return false;
    }
    else
    {
      ptr_wrapper_->registerUserPackageUnpackCallback(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_100HZ), VehicleNode::publish100HzData, this);
    }
  }

  std::vector<Telemetry::TopicName> topicList50Hz;
  // 50 Hz package from FC
  topicList50Hz.push_back(Telemetry::TOPIC_GPS_FUSED);
  topicList50Hz.push_back(Telemetry::TOPIC_ALTITUDE_FUSIONED);
  topicList50Hz.push_back(Telemetry::TOPIC_HEIGHT_FUSION);
  topicList50Hz.push_back(Telemetry::TOPIC_STATUS_FLIGHT);
  topicList50Hz.push_back(Telemetry::TOPIC_STATUS_DISPLAYMODE);
  topicList50Hz.push_back(Telemetry::TOPIC_GIMBAL_ANGLES);
  topicList50Hz.push_back(Telemetry::TOPIC_GIMBAL_STATUS);
  topicList50Hz.push_back(Telemetry::TOPIC_ESC_DATA);

  // acturally gimbal data is from Gimbal directly
  if (ptr_wrapper_->isM300())
  {
    topicList50Hz.push_back(Telemetry::TOPIC_THREE_GIMBAL_DATA);
    std::cout << "Pushing TOPIC_THREE_GIMBAL_DATA 2\n";
  }
  else
  {
    topicList50Hz.push_back(Telemetry::TOPIC_DUAL_GIMBAL_DATA);
  }
  topicList50Hz.push_back(Telemetry::TOPIC_RC);
  topicList50Hz.push_back(Telemetry::TOPIC_VELOCITY);
  topicList50Hz.push_back(Telemetry::TOPIC_GPS_CONTROL_LEVEL);

  if(ptr_wrapper_->getFwVersion() > versionBase33)
  {
    topicList50Hz.push_back(Telemetry::TOPIC_POSITION_VO);
    topicList50Hz.push_back(Telemetry::TOPIC_RC_WITH_FLAG_DATA);
    topicList50Hz.push_back(Telemetry::TOPIC_FLIGHT_ANOMALY);

    // A3 and N3 has access to more buttons on RC
    std::string hardwareVersion(ptr_wrapper_->getHwVersion());
    if( (hardwareVersion == std::string(Version::N3)) || hardwareVersion == std::string(Version::A3))
    {
      topicList50Hz.push_back(Telemetry::TOPIC_RC_FULL_RAW_DATA);
    }

    // Advertise rc connection status only if this topic is supported by FW
    rc_connection_status_publisher_ = nh_.advertise<std_msgs::UInt8>(publishName+"/rc_connection_status", 10);
    flight_anomaly_publisher_ = nh_.advertise<dji_sdk::FlightAnomaly>(publishName+"/flight_anomaly", 10);
  }

  int nTopic50Hz    = topicList50Hz.size();
  if (ptr_wrapper_->initPackageFromTopicList(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_50HZ), nTopic50Hz,
                                                   topicList50Hz.data(), 1, 50))
  {
    ack = ptr_wrapper_->startPackage(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_50HZ), WAIT_TIMEOUT);
    if (ACK::getError(ack))
    {
      ptr_wrapper_->removePackage(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_50HZ), WAIT_TIMEOUT);
      ROS_ERROR("Failed to start 50Hz package");
      return false;
    }
    else
    {
      ptr_wrapper_->registerUserPackageUnpackCallback(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_50HZ), VehicleNode::publish50HzData, (UserData) this);
    }
  }




  //! Check if RTK is supported in the FC
  Telemetry::TopicName topicRTKSupport[] = { Telemetry::TOPIC_RTK_POSITION  };

  int nTopicRTKSupport    = sizeof(topicRTKSupport)/sizeof(topicRTKSupport[0]);
  if (ptr_wrapper_->initPackageFromTopicList(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_5HZ), nTopicRTKSupport,
                                             topicRTKSupport, 1, 5))
  {
    ack = ptr_wrapper_->startPackage(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_5HZ), WAIT_TIMEOUT);
    if (ack.data == ErrorCode::SubscribeACK::SOURCE_DEVICE_OFFLINE)
    {
      rtk_support_ = false;
      ROS_INFO("Flight Controller does not support RTK");
    }
    else
    {
      rtk_support_ = true;
      ptr_wrapper_->removePackage(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_5HZ), WAIT_TIMEOUT);
    }
  }

  std::vector<Telemetry::TopicName> topicList5hz;
  topicList5hz.push_back(Telemetry::TOPIC_GPS_DATE);
  topicList5hz.push_back(Telemetry::TOPIC_GPS_TIME);
  topicList5hz.push_back(Telemetry::TOPIC_GPS_POSITION);
  topicList5hz.push_back(Telemetry::TOPIC_GPS_VELOCITY);
  topicList5hz.push_back(Telemetry::TOPIC_GPS_DETAILS);
  topicList5hz.push_back(Telemetry::TOPIC_BATTERY_INFO);

//  std::string hardwareVersion(ptr_wrapper_->getHwVersion());
//  if (hardwareVersion == "PM430") {
//      std::cout << "Enabled Telemetry::TOPIC_RC_FULL_RAW_DATA for " << hardwareVersion << "\n";
//      std::cout << "topic list 5hz size:\t" << topicList5hz.size() << "\n";
//      topicList5hz.push_back(Telemetry::TOPIC_RC_FULL_RAW_DATA);
//      std::cout << "topic list 5hz size:\t" << topicList5hz.size() << "\n";
//  }




  if(rtk_support_)
  {
    topicList5hz.push_back(Telemetry::TOPIC_RTK_POSITION);
    topicList5hz.push_back(Telemetry::TOPIC_RTK_VELOCITY);
    topicList5hz.push_back(Telemetry::TOPIC_RTK_YAW);
    topicList5hz.push_back(Telemetry::TOPIC_RTK_YAW_INFO);
    topicList5hz.push_back(Telemetry::TOPIC_RTK_POSITION_INFO);

    // Advertise rtk data only when rtk is supported
    rtk_position_publisher_ = nh_.advertise<sensor_msgs::NavSatFix>(publishName+"/rtk_position", 10);
    rtk_velocity_publisher_ = nh_.advertise<geometry_msgs::Vector3Stamped>(publishName+"/rtk_velocity", 10);
    rtk_yaw_publisher_ = nh_.advertise<std_msgs::Int16>(publishName+"/rtk_yaw", 10);
    rtk_position_info_publisher_ = nh_.advertise<std_msgs::UInt8>(publishName+"/rtk_info_position", 10);
    rtk_yaw_info_publisher_ = nh_.advertise<std_msgs::UInt8>(publishName+"/rtk_info_yaw", 10);

    if(ptr_wrapper_->getFwVersion() > versionBase33)
    {
      topicList5hz.push_back(Telemetry::TOPIC_RTK_CONNECT_STATUS);

      // Advertise rtk connection only when rtk is supported
      rtk_connection_status_publisher_ = nh_.advertise<std_msgs::UInt8>(publishName+"/rtk_connection_status", 10);
    }
  }

  int nTopic5hz    = topicList5hz.size();
  if (ptr_wrapper_->initPackageFromTopicList(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_5HZ), nTopic5hz,
                                             topicList5hz.data(), 1, 5))
  {
    ack = ptr_wrapper_->startPackage(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_5HZ), WAIT_TIMEOUT);
    if (ACK::getError(ack))
    {
      ptr_wrapper_->removePackage(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_5HZ), WAIT_TIMEOUT);
      ROS_ERROR("Failed to start 5hz package");
      return false;
    }
    else
    {
      ptr_wrapper_->registerUserPackageUnpackCallback(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_5HZ), VehicleNode::publish5HzData, (UserData) this);
    }
  }

  // 400 Hz data from FC
  std::vector<Telemetry::TopicName> topicList400Hz;
  topicList400Hz.push_back(Telemetry::TOPIC_HARD_SYNC);

  int nTopic400Hz = topicList400Hz.size();
  if (ptr_wrapper_->initPackageFromTopicList(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_400HZ), nTopic400Hz,
                                             topicList400Hz.data(), 1, 400))
  {
    ack = ptr_wrapper_->startPackage(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_400HZ), WAIT_TIMEOUT);
    if(ACK::getError(ack))
    {
      ptr_wrapper_->removePackage(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_400HZ), WAIT_TIMEOUT);
      ROS_ERROR("Failed to start 400Hz package");
      return false;
    }
    else
    {
      ptr_wrapper_->registerUserPackageUnpackCallback(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_400HZ), VehicleNode::publish400HzData, this);
    }
  }

  ros::Duration(1).sleep();
  return true;
}

bool VehicleNode::cleanUpSubscribeFromFC()
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }
  Vehicle* vehicle = ptr_wrapper_->getVehicle();

  ptr_wrapper_->removePackage(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_1HZ), WAIT_TIMEOUT);
  ptr_wrapper_->removePackage(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_5HZ), WAIT_TIMEOUT);
  ptr_wrapper_->removePackage(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_50HZ), WAIT_TIMEOUT);
  ptr_wrapper_->removePackage(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_100HZ), WAIT_TIMEOUT);
  ptr_wrapper_->removePackage(static_cast<int>(SubscribePackgeIndex::PACKAGE_ID_400HZ), WAIT_TIMEOUT);
  if (vehicle->hardSync)
  {
    ptr_wrapper_->unsubscribeNMEAMsgs();
    ptr_wrapper_->unsubscribeUTCTime();
    ptr_wrapper_->unsubscribeFCTimeInUTCRef();
    ptr_wrapper_->unsubscribePPSSource();
  }
  return true;
}

vector<string> VehicleNode::getClasses()
{
    return classes;
}

Net VehicleNode::getNetwork()
{
    return net;
}

vector<string> VehicleNode::getClassesHarpy()
{
    return classes_harpy;
}

Net VehicleNode::getNetworkHarpy()
{
    return net_harpy;
}

KalmanFilterCustom* VehicleNode::getKalmanFilter()
{
    return &kf;
}

//Ptr<Tracker> VehicleNode::getTracker()
//{
//    return tracker;
//}

//void VehicleNode::setTracker(Ptr<Tracker> newTracker)
//{
//    tracker = newTracker;
//}

PID* VehicleNode::getPIDx()
{
	return &controlPIDx;
}

PID* VehicleNode::getPIDy()
{
	return &controlPIDy;
}

bool VehicleNode::isM300()
{
  return ptr_wrapper_->isM300();
}

VehicleNode::VehicleNode(int test)
{
  initService();
}

#ifdef ADVANCED_SENSING
bool VehicleNode::setupCameraStreamCallback(dji_sdk::SetupCameraStream::Request& request, dji_sdk::SetupCameraStream::Response& response)
{
  ROS_DEBUG("called cameraStreamCallback");
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    response.result = false;
  }

  ptr_wrapper_->setAcmDevicePath(device_acm_);

  if(request.cameraType == request.FPV_CAM)
  {
    if(request.start == 1)
    {
      response.result = ptr_wrapper_->startFPVCameraStream(&publishFPVCameraImage, this);
    }
    else
    {
      response.result = ptr_wrapper_->stopFPVCameraStream();
    }
  }
  else if(request.cameraType == request.MAIN_CAM)
  {
    if(request.start == 1)
    {
      if(!streaming){
	      PayloadIndexType index = static_cast<PayloadIndexType>(0);
	      ptr_wrapper_->getVehicle()->cameraManager->setModeSync(index, CameraModule::WorkMode::RECORD_VIDEO, 3);
	      response.result = ptr_wrapper_->startMainCameraStream(&publishMainCameraImage, this);
	//DJI::OSDK::LiveView::LiveViewErrCode error = ptr_wrapper_->changeH264Source(static_cast<DJI::OSDK::LiveView::LiveViewCameraPosition>(0), static_cast<DJI::OSDK::LiveView::LiveViewCameraSource>(2))
//    response.result = ptr_wrapper_->startH264Stream(static_cast<DJI::OSDK::LiveView::LiveViewCameraPosition>(0), &publishCameraH264, this);
	      if(response.result)
		streaming = true;
      }else{
      	response.result = true;
      }
    }
    else
    {
      if(streaming){
	      stopDetector();
	      response.result = ptr_wrapper_->stopMainCameraStream();
    //response.result = ptr_wrapper_->stopH264Stream(static_cast<DJI::OSDK::LiveView::LiveView::LiveViewCameraPosition>(0));
	      if(response.result)
		streaming = false;
      }else{
      	response.result = true;
      }
    }
  }

  return response.result;
}

bool VehicleNode::stopKalmanTrackCallback(dji_sdk::OdroidPowerOff::Request& request, dji_sdk::OdroidPowerOff::Response& response)
{
    ROS_DEBUG("called stopKalmanTrackCallback");
    bool result = false;

    if(request.command == 1){
        dji_sdk::BoundingBoxes boundBoxes;
        boundBoxes.header.stamp = ros::Time::now();
        boundBoxes.header.frame_id = "MAIN_CAMERA";
        boundBoxes.labels.resize(0);
        boundBoxes.confidences.resize(0);
        boundBoxes.tops.resize(0);
        boundBoxes.lefts.resize(0);
        boundBoxes.rights.resize(0);
        boundBoxes.bottoms.resize(0);
        VehicleNode::bounding_boxes_publisher_.publish(boundBoxes); 
        stopDetector();
        result = true;
    }

    response.result = result;
    return true;
}

bool VehicleNode::setupCameraH264Callback(dji_sdk::SetupCameraH264::Request& request, dji_sdk::SetupCameraH264::Response& response)
{
  ROS_DEBUG("called camerah264Callback");
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    response.result = false;
  }

  ptr_wrapper_->setAcmDevicePath(device_acm_);

  if (request.start == 1)
  {
    response.result = ptr_wrapper_->startH264Stream(static_cast<DJI::OSDK::LiveView::LiveViewCameraPosition>(request.request_view), &publishCameraH264, this);
  }
  else
  {
    response.result = ptr_wrapper_->stopH264Stream(static_cast<DJI::OSDK::LiveView::LiveView::LiveViewCameraPosition>(request.request_view));
  }

  return response.result;
}

// Uses Stereo240pSubscription
bool VehicleNode::stereo240pSubscriptionCallback(dji_sdk::Stereo240pSubscription::Request&  request, dji_sdk::Stereo240pSubscription::Response& response)
{
  std::cout << "Hello i am deep in stereo240pSubscriptionCallback\n";
  ROS_DEBUG("called stereo240pSubscriptionCallback");

  if(ptr_wrapper_ == nullptr)
  {
      ROS_ERROR_STREAM("Vehicle modules is nullptr");
      return false;
  } else {
    //std::cout << "ptr_wrapper_:" << ptr_wrapper_ << "\n";//reinterpret_cast<void *>(ptr_wrapper_) << "\n";
  }
  std::cout << "ptr_wrapper_ ok\n";

  if (request.unsubscribe_240p == 1)
  {
    ptr_wrapper_->unsubscribeStereoImages();
    response.result = true;
    ROS_INFO("unsubscribe stereo 240p images");
    return true;
  }

  dji_sdk::ImageSelection image_select;
  memset(&image_select, 0, sizeof(dji_sdk::ImageSelection));
  //std::cout << "image_select:" << image_select << "\n"; //reinterpret_cast<void *>(image_select) << "\n";
  //std::cout << "publish240pStereoImage:" << publish240pStereoImage << "\n"; //reinterpret_cast<void *>(publish240pStereoImage) << "\n";
  std::cout << "memset ok\n";

  if (request.front_right_240p == 1)
    image_select.front_right = 1;

  if (request.front_left_240p == 1)
    image_select.front_left = 1;

  if (request.down_front_240p == 1)
    image_select.down_front = 1;

  if (request.down_back_240p == 1)
    image_select.down_back = 1;

  std::cout << "image_select ok\n";

  this->stereo_subscription_success = false;
  this->
  ptr_wrapper_->subscribeStereoImages(&image_select, &publish240pStereoImage, this);
  std::cout << "subscribeStereoImages ok\n";

  ros::Duration(1).sleep();

  if (this->stereo_subscription_success == true)
  {
    response.result = true;
  }
  else
  {
    response.result = false;
    ROS_WARN("Stereo 240p subscription service failed, please check your request content.");
  }

  std::cout << "stereo_subscription_success ok\t" << "response.result:" << response.result << "\n";
  

  return true;
}

bool
VehicleNode::stereoDepthSubscriptionCallback(dji_sdk::StereoDepthSubscription::Request&  request, dji_sdk::StereoDepthSubscription::Response& response)
{
  std::cout << "Hello i am deep in stereoDepthSubscriptionCallback\n";
  ROS_DEBUG("called stereoDepthSubscriptionCallback");

  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  if (request.unsubscribe_240p == 1)
  {
    ptr_wrapper_->unsubscribeStereoImages();
    response.result = true;
    ROS_INFO("unsubscribe stereo 240p images");
    return true;
  }

  if (request.front_depth_240p == 1)
  {
    this->stereo_subscription_success = false;
    std::cout << "subscribeFrontStereoDisparity\n";
    if (!isM300()) {
      ptr_wrapper_->subscribeFrontStereoDisparity(&publish240pStereoImage, this);
    } else {
      std::cout << "isM300\n";



      VehicleStereoImagePacketType *userData = new VehicleStereoImagePacketType;

      StereoImagePacketType *pack = new StereoImagePacketType;
      pack->info = { 0 };
      pack->imageRawBuffer = NULL;
      pack->mutex = NULL;
      pack->gotData = false;

      OsdkOsal_MutexCreate(&pack->mutex);
      if (pack->mutex) {
        std::cout << "stereoImagePacket->mutex pointer valid\n";
      }

      userData->stereoImagePacketType = pack;
      userData->vh_node = this;
    
   

      //OsdkOsal_MutexUnlock(stereoImagePacket->mutex);
      //std::cout << "unlocked mutex stereoImagePacket->mutex\n";


      ptr_wrapper_->subscribeFrontStereoDisparityM300(&PerceptionImageCB, userData);

      //OsdkOsal_MutexDestroy(stereoImagePacket.mutex);
    }
    std::cout << "subscribeFrontStereoDisparity DOne\n";
  }
  else
  {
    ROS_WARN("no depth image is subscribed");
    return true;
  }

  ros::Duration(1).sleep();

  if (this->stereo_subscription_success == true)
  {
    response.result = true;
  }
  else
  {
    response.result = false;
    ROS_WARN("Stereo 240p subscription service failed, please check your request content.");
  }

  std::cout << "stereoDepthSubscriptionCallback SUCCESS\n";
  return true;
}

bool VehicleNode::stereoVGASubscriptionCallback(dji_sdk::StereoVGASubscription::Request&  request, dji_sdk::StereoVGASubscription::Response& response)
{
  ROS_INFO("called stereoVGASubscriptionCallback");

  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  if (request.unsubscribe_vga == 1)
  {
    ptr_wrapper_->unsubscribeVGAImages();
    response.result = true;
    ROS_INFO("unsubscribe stereo vga images");
    return true;
  }

  if (request.vga_freq != request.VGA_20_HZ
      && request.vga_freq != request.VGA_10_HZ)
  {
    ROS_ERROR("VGA subscription frequency is wrong");
    response.result = false;
    return true;
  }

  if (request.front_vga == 1)
  {
    this->stereo_vga_subscription_success = false;
    std::cout << "subscribeFrontStereoVGA calling...\n";
    ptr_wrapper_->subscribeFrontStereoVGA(request.vga_freq, &publishVGAStereoImage, this);
    ros::Duration(2).sleep();
    std::cout << "this->stereo_vga_subscription_success:" << this->stereo_vga_subscription_success << "\n";
    std::cout << "subscribeFrontStereoVGA done\n";
    //this->stereo_vga_subscription_success = true;
  }

  if (this->stereo_vga_subscription_success == true)
  {
    response.result = true;
    ROS_INFO("Stereo VGA subscription service success");
  }
  else
  {
    response.result = false;
    ROS_WARN("Stereo VGA subscription service failed, please check your request content.");
  }

  return true;
}

void VehicleNode::scanAreaResponseCallback(const dji_sdk::MissionWaypointTask::ConstPtr& mission)
{

    ROS_DEBUG("called scanAreaResponseCallback");
    ros::spinOnce();

    size_t numWaypoints = mission->mission_waypoint.size();

    ROS_WARN("Number of WayPoints: %i", numWaypoints);

    int responseTimeout = 20;

    // Waypoint Mission : Initialization
    //kios::MissionWaypointTask2 waypointTask;
    dji_sdk::InitWaypointV2Setting initWaypointV2Setting_;
    DJI::OSDK::WaypointV2 waypointV2Vector;
    initWaypointV2Setting_.request.polygonNum = 0;
    initWaypointV2Setting_.request.radius = 10;
    initWaypointV2Setting_.request.actionNum = 0;
    
    dji_sdk::GenerateWaypointV2Action generateWaypointV2Action_;
    dji_sdk::WaypointV2Action actionVector;

    for (uint16_t i = 0; i < initWaypointV2Setting_.request.actionNum; i++)
    {
      actionVector.actionId  = i;
      actionVector.waypointV2ActionTriggerType  = dji_sdk::WaypointV2Action::DJIWaypointV2ActionTriggerTypeSampleReachPoint;
      actionVector.waypointV2SampleReachPointTrigger.waypointIndex = i;
      actionVector.waypointV2SampleReachPointTrigger.terminateNum = 0;
      actionVector.waypointV2ACtionActuatorType = dji_sdk::WaypointV2Action::DJIWaypointV2ActionActuatorTypeCamera;
      actionVector.waypointV2CameraActuator.actuatorIndex = 0;
      actionVector.waypointV2CameraActuator.DJIWaypointV2ActionActuatorCameraOperationType = dji_sdk::WaypointV2CameraActuator::DJIWaypointV2ActionActuatorCameraOperationTypeTakePhoto;
      generateWaypointV2Action_.request.actions.push_back(actionVector);
    }
    waypointV2GenerateActionsCallback(generateWaypointV2Action_.request, generateWaypointV2Action_.response);

    initWaypointV2Setting_.request.waypointV2InitSettings.repeatTimes = mission->mission_exec_times;
    initWaypointV2Setting_.request.waypointV2InitSettings.finishedAction = 0; //mission->action_on_finish;
    initWaypointV2Setting_.request.waypointV2InitSettings.maxFlightSpeed = mission->velocity_range;
    initWaypointV2Setting_.request.waypointV2InitSettings.autoFlightSpeed = mission->idle_velocity;
    initWaypointV2Setting_.request.waypointV2InitSettings.exitMissionOnRCSignalLost = 0; //mission->action_on_rc_lost;
    initWaypointV2Setting_.request.waypointV2InitSettings.gotoFirstWaypointMode = 0; //initWaypointV2Setting_.request.waypointV2InitSettings.DJIWaypointV2MissionGotoFirstWaypointModeSafely;
    initWaypointV2Setting_.request.waypointV2InitSettings.missTotalLen = mission->mission_waypoint.size();


    std::cout << "###\tmission\t###\n";
    std::cout << "repeatTimes:\t" << mission->mission_exec_times << "\n";
    std::cout << "finishedAction:\t" << mission->action_on_finish << "\n";
    std::cout << "maxFlightSpeed:\t" << mission->velocity_range << "\n";
    std::cout << "autoFlightSpeed:\t" << mission->idle_velocity << "\n";
    std::cout << "exitMissionOnRCSignalLost:\t" << mission->action_on_rc_lost << "\n";
    std::cout << "gotoFirstWaypointMode:\t" << initWaypointV2Setting_.request.waypointV2InitSettings.DJIWaypointV2MissionGotoFirstWaypointModeSafely << "\n";
    std::cout << "missTotalLen:\t" << mission->mission_waypoint.size() << "\n";


    std::cout << "### \tinitWaypointV2Setting_\t ###\n";
    std::cout << "repeatTimes:\t" << initWaypointV2Setting_.request.waypointV2InitSettings.repeatTimes << "\n";
    std::cout << "finishedAction:\t" << initWaypointV2Setting_.request.waypointV2InitSettings.finishedAction << "\n";
    std::cout << "maxFlightSpeed:\t" << initWaypointV2Setting_.request.waypointV2InitSettings.maxFlightSpeed << "\n";
    std::cout << "autoFlightSpeed:\t" << initWaypointV2Setting_.request.waypointV2InitSettings.autoFlightSpeed << "\n";
    std::cout << "exitMissionOnRCSignalLost:\t" << initWaypointV2Setting_.request.waypointV2InitSettings.gotoFirstWaypointMode << "\n";
    std::cout << "gotoFirstWaypointMode:\t" << initWaypointV2Setting_.request.waypointV2InitSettings.gotoFirstWaypointMode << "\n";
    std::cout << "missTotalLen:\t" << initWaypointV2Setting_.request.waypointV2InitSettings.missTotalLen << "\n";

    std::vector<dji_sdk::WaypointV2> waypointList;
    dji_sdk::WaypointV2 waypointV2;


    // Iterative algorithm
    for (int i = 0; i < mission->mission_waypoint.size(); i++) {
            waypointV2.waypointType = dji_sdk::DJIWaypointV2FlightPathModeGoToPointInAStraightLineAndStop;
            waypointV2.headingMode = dji_sdk::DJIWaypointV2HeadingModeAuto;
	    waypointV2.config.useLocalCruiseVel = 0;
	    waypointV2.config.useLocalMaxVel = 0;

	    waypointV2.dampingDistance = mission->mission_waypoint[i].damping_distance;
	    waypointV2.heading = 0;
  	    waypointV2.turnMode = dji_sdk::DJIWaypointV2TurnModeClockwise;

	    waypointV2.positionX = 0;
	    waypointV2.positionY = 0;
	    waypointV2.positionZ = 0;
	    waypointV2.maxFlightSpeed= mission->velocity_range;
	    waypointV2.autoFlightSpeed = mission->idle_velocity;
	    waypointV2.latitude = mission->mission_waypoint[i].latitude * (C_PI/180.0);
	    waypointV2.longitude = mission->mission_waypoint[i].longitude * (C_PI/180.0);
	    waypointV2.relativeHeight = mission->mission_waypoint[i].altitude;
	    waypointList.push_back(waypointV2);
            ROS_INFO("Waypoint created at (LLA): %f \t%f \t%f\n",  waypointV2.latitude, waypointV2.longitude , waypointV2.relativeHeight);
    }

    initWaypointV2Setting_.request.waypointV2InitSettings.mission = waypointList;


    // Waypoint Mission: Init mission
    ROS_INFO("Initializing Waypoint Mission..\n");

    waypointV2InitSettingCallback(initWaypointV2Setting_.request, initWaypointV2Setting_.response);


    ROS_INFO("Uploading Waypoint Mission..\n");
    dji_sdk::UploadWaypointV2Mission uploadWaypointV2Mission_;
    waypointV2UploadMissionCallback(uploadWaypointV2Mission_.request, uploadWaypointV2Mission_.response);
 
    if (!uploadWaypointV2Mission_.response.result)
    {
        ROS_WARN("result %s", uploadWaypointV2Mission_.response.result);
    }

    if (uploadWaypointV2Mission_.response.result)
    {
        ROS_INFO("Waypoint upload command sent successfully");
        std_msgs::String msg;
        msg.data = "Waypoint upload command sent successfully";
        VehicleNode::mission_waypoint_upload_report_publisher_.publish(msg);

    }
    else
    {
        ROS_WARN("Failed sending waypoint upload command");
        return;
    }
}

/*
void VehicleNode::trackCarCallback(const std_msgs::Float32MultiArray::ConstPtr& box)
{
    ROS_DEBUG("called trackCarCallback");
    ros::spinOnce();
    std::cout << std::setprecision(6) << std::fixed;
    std::cout << "TRACK BOX NEAREST TO X: " << box.get()->data[0] << " Y: " << box.get()->data[1] << std::endl;
    nearestBox(box.get()->data[0], box.get()->data[1]);
}
*/
#ifdef OPEN_CV_INSTALLED
/*
bool VehicleNode::getM300StereoParamsCallback(dji_sdk::GetM300StereoParams::Request& request, 
                                              dji_sdk::GetM300StereoParams::Response& response)
{
  ROS_INFO("called getM300StereoParamsCallback");

  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    response.result = false;
  }

  Vehicle* vehicle = ptr_wrapper_->getVehicle();
  M300StereoParamTool *tool = new M300StereoParamTool(vehicle);
  Perception::CamParamType stereoParam =
      tool->getM300stereoParams(Perception::DirectionType::RECTIFY_FRONT);
  if (tool->createStereoParamsYamlFile(M300_FRONT_STEREO_PARAM_YAML_NAME, stereoParam))
  {
    tool->setParamFileForM300(M300_FRONT_STEREO_PARAM_YAML_NAME);
    response.result = true;
  }
  else
  {
    response.result = false;
  }

  return response.result;
}*/
#endif
#endif

bool VehicleNode::getDroneTypeCallback(dji_sdk::GetDroneType::Request &request, dji_sdk::GetDroneType::Response &response)
{
  ROS_DEBUG("called getDroneTypeCallback");

  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  if (ptr_wrapper_->isM100())
  {
    response.drone_type = static_cast<uint8_t>(dji_sdk::Dronetype::M100);
    return true;
  }
  else if (ptr_wrapper_->isM200V2())
  {
    response.drone_type = static_cast<uint8_t>(dji_sdk::Dronetype::M210V2);
    return true;
  }
  else if (ptr_wrapper_->isM300())
  {
    response.drone_type = static_cast<uint8_t>(dji_sdk::Dronetype::M300);
    return true;
  }
  else if (ptr_wrapper_->isM600())
  {
    response.drone_type = static_cast<uint8_t>(dji_sdk::Dronetype::M600);
    return true;
  }
  else
  {
    response.drone_type = static_cast<uint8_t>(dji_sdk::Dronetype::INVALID_TYPE);
    return false;
  }

}

bool VehicleNode::taskCtrlCallback(FlightTaskControl::Request&  request, FlightTaskControl::Response& response)
{
  ROS_DEBUG("called taskCtrlCallback");
  response.result = false;
  ACK::ErrorCode ack;
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  switch (request.task)
  {
    case FlightTaskControl::Request::TASK_GOHOME:
     {
        ROS_INFO_STREAM("call go home service");
        if (ptr_wrapper_->goHome(FLIGHT_CONTROL_WAIT_TIMEOUT))
        {
          response.result = true;
        }
        break;
      }
    case FlightTaskControl::Request::TASK_GOHOME_AND_CONFIRM_LANDING:
      {
        ROS_INFO_STREAM("call go home and confirm landing service");
        if (ptr_wrapper_->goHomeAndConfirmLanding(FLIGHT_CONTROL_WAIT_TIMEOUT))
        {
            response.result = true;
        }
        break;
      }
    case FlightTaskControl::Request::TASK_POSITION_AND_YAW_CONTROL:
      {
        ROS_INFO_STREAM("call move local position offset service");
        dji_sdk::JoystickCommand joystickCommand;
        joystickCommand.x   = request.joystickCommand.x;
        joystickCommand.y   = request.joystickCommand.y;
        joystickCommand.z   = request.joystickCommand.z;
        joystickCommand.yaw = request.joystickCommand.yaw;

        if (ptr_wrapper_->moveByPositionOffset(joystickCommand, FLIGHT_CONTROL_WAIT_TIMEOUT,
                                               request.posThresholdInM, request.yawThresholdInDeg))
        {
          response.result = true;
        }
        break;
      }
    case FlightTaskControl::Request::TASK_TAKEOFF:
      {
        ROS_INFO_STREAM("call takeoff service");
        if (ptr_wrapper_->monitoredTakeoff(FLIGHT_CONTROL_WAIT_TIMEOUT))
        {
          response.result = true;
        }
        break;
      }
    case FlightTaskControl::Request::TASK_VELOCITY_AND_YAWRATE_CONTROL:
      {
        ROS_INFO_STREAM("call velocity and yaw rate service");

        dji_sdk::JoystickCommand joystickCommand;
        joystickCommand.x   = request.joystickCommand.x;
        joystickCommand.y   = request.joystickCommand.y;
        joystickCommand.z   = request.joystickCommand.z;
        joystickCommand.yaw = request.joystickCommand.yaw;

        ptr_wrapper_->velocityAndYawRateCtrl(joystickCommand, request.velocityControlTimeMs);
        response.result = true;

        break;
      }
    case FlightTaskControl::Request::TASK_LAND:
      {
        ROS_INFO_STREAM("call land service");
        if (ptr_wrapper_->monitoredLanding(FLIGHT_CONTROL_WAIT_TIMEOUT))
        {
          response.result = true;
        }
        break;
      }
    case FlightTaskControl::Request::START_MOTOR:
    {
        ROS_INFO_STREAM("call start motor service");
        if (ptr_wrapper_->turnOnOffMotors(true))
        {
          response.result = true;
        }
        break;
    }
    case FlightTaskControl::Request::STOP_MOTOR:
    {
        ROS_INFO_STREAM("call stop motor service");
        if (ptr_wrapper_->turnOnOffMotors(false))
        {
          response.result = true;
        }
        break;
    }
    case FlightTaskControl::Request::TASK_EXIT_GO_HOME:
      {
        ROS_INFO_STREAM("call cancel go home service");
        if (ptr_wrapper_->cancelGoHome(FLIGHT_CONTROL_WAIT_TIMEOUT))
        {
          response.result = true;
        }
        break;
      }
    case FlightTaskControl::Request::TASK_EXIT_LANDING:
      {
        ROS_INFO_STREAM("call cancel landing service");
        if (ptr_wrapper_->cancelLanding(FLIGHT_CONTROL_WAIT_TIMEOUT))
        {
          response.result = true;
        }
        break;
      }
    case FlightTaskControl::Request::TASK_FORCE_LANDING_AVOID_GROUND:
    {
        ROS_INFO_STREAM("call confirm landing service");
        if (ptr_wrapper_->startConfirmLanding(FLIGHT_CONTROL_WAIT_TIMEOUT))
        {
          response.result = true;
        }
        break;
    }
    case FlightTaskControl::Request::TASK_FORCE_LANDING:
    {
        ROS_INFO_STREAM("call force landing service");
        if (ptr_wrapper_->startForceLanding(FLIGHT_CONTROL_WAIT_TIMEOUT))
        {
          response.result = true;
        }
        break;
    }
    default:
      {
        ROS_INFO_STREAM("No recognized task");
        response.result = false;
        break;
      }
  }

  if (response.result)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool VehicleNode::setJoystickModeCallback(SetJoystickMode::Request& request, SetJoystickMode::Response& response)
{
  ROS_DEBUG("called setJoystickModeCallback");
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  dji_sdk::JoystickMode joystickMode;
  joystickMode.horizontalLogic = request.horizontal_mode;
  joystickMode.verticalLogic   = request.vertical_mode;
  joystickMode.yawLogic        = request.yaw_mode;
  joystickMode.horizontalCoordinate = request.horizontal_coordinate;
  joystickMode.stableMode      = request.stable_mode;

  ptr_wrapper_->setJoystickMode(joystickMode);

  response.result = true;
  return true;
}

bool VehicleNode::JoystickActionCallback(JoystickAction::Request& request, JoystickAction::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  dji_sdk::JoystickCommand joystickCommand;
  joystickCommand.x = request.joystickCommand.x;
  joystickCommand.y = request.joystickCommand.y;
  joystickCommand.z = request.joystickCommand.z;
  joystickCommand.yaw = request.joystickCommand.yaw;

  //std::cout << "Received Joystick:\n"  << joystickCommand.x << "\t" << joystickCommand.y << "\t" << joystickCommand.z << "\t" << joystickCommand.yaw << "\n";

  ptr_wrapper_->JoystickAction(joystickCommand);

  response.result = true;
  return true;
}

bool VehicleNode::gimbalCtrlCallback(GimbalAction::Request& request, GimbalAction::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }
  response.result = false;
  ROS_INFO("Current gimbal %d, angle (p,r,y) = (%0.2f, %0.2f, %0.2f)", static_cast<int>(request.payload_index),
           ptr_wrapper_->getGimbalData(static_cast<PayloadIndex>(request.payload_index)).pitch,
           ptr_wrapper_->getGimbalData(static_cast<PayloadIndex>(request.payload_index)).roll,
           ptr_wrapper_->getGimbalData(static_cast<PayloadIndex>(request.payload_index)).yaw);
  ROS_INFO_STREAM("Call gimbal Ctrl.");

  if (request.is_reset)
  {
    response.result = ptr_wrapper_->resetGimbal(static_cast<PayloadIndex>(request.payload_index));
  }
  else
  {
    GimbalRotationData gimbalRotationData;
    gimbalRotationData.rotationMode = request.rotationMode;
    gimbalRotationData.pitch = request.pitch;
    gimbalRotationData.roll  = request.roll;
    gimbalRotationData.yaw   = request.yaw;
    gimbalRotationData.time  = request.time;
    response.result = ptr_wrapper_->rotateGimbal(static_cast<PayloadIndex>(request.payload_index), gimbalRotationData);
  }

//  sleep(2);
//  ROS_INFO("Current gimbal %d , angle (p,r,y) = (%0.2f, %0.2f, %0.2f)", request.payload_index,
//           ptr_wrapper_->getGimbalData(static_cast<PayloadIndex>(request.payload_index)).pitch,
//           ptr_wrapper_->getGimbalData(static_cast<PayloadIndex>(request.payload_index)).roll,
//           ptr_wrapper_->getGimbalData(static_cast<PayloadIndex>(request.payload_index)).yaw);
  return true;
}

bool VehicleNode::cameraSetEVCallback(CameraEV::Request& request, CameraEV::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }
  response.result = true;
  response.result &= ptr_wrapper_->setExposureMode(static_cast<PayloadIndex>(request.payload_index),
                                                   static_cast<ExposureMode>(request.exposure_mode));
  response.result &= ptr_wrapper_->setEV(static_cast<PayloadIndex>(request.payload_index),
                                         static_cast<ExposureCompensation>(request.exposure_compensation));
  return response.result;
}

bool VehicleNode::cameraSetShutterSpeedCallback(CameraShutterSpeed::Request& request, CameraShutterSpeed::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }
  response.result = true;
  response.result &= ptr_wrapper_->setExposureMode(static_cast<PayloadIndex>(request.payload_index),
                                                   static_cast<ExposureMode>(request.exposure_mode));
  response.result &= ptr_wrapper_->setShutterSpeed(static_cast<PayloadIndex>(request.payload_index),
                                                   static_cast<ShutterSpeed>(request.shutter_speed));
  return response.result;
}

bool VehicleNode::cameraSetApertureCallback(CameraAperture::Request& request, CameraAperture::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }
  response.result = true;
  response.result &= ptr_wrapper_->setExposureMode(static_cast<PayloadIndex>(request.payload_index),
                                                   static_cast<ExposureMode>(request.exposure_mode));
  response.result &= ptr_wrapper_->setAperture(static_cast<PayloadIndex>(request.payload_index),
                                               static_cast<Aperture>(request.aperture));
  return response.result;
}

bool VehicleNode::cameraSetISOCallback(CameraISO::Request& request, CameraISO::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }
  response.result = true;
  response.result &= ptr_wrapper_->setExposureMode(static_cast<PayloadIndex>(request.payload_index),
                                                   static_cast<ExposureMode>(request.exposure_mode));
  response.result &= ptr_wrapper_->setISO(static_cast<PayloadIndex>(request.payload_index),
                                          static_cast<ISO>(request.iso_data));
  return response.result;
}

bool VehicleNode::cameraSetFocusPointCallback(CameraFocusPoint::Request& request, CameraFocusPoint::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }
  response.result = ptr_wrapper_->setFocusPoint(static_cast<PayloadIndex>(request.payload_index), request.x, request.y);
  return response.result;
}

bool VehicleNode::cameraSetTapZoomPointCallback(CameraTapZoomPoint::Request& request, CameraTapZoomPoint::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }
  response.result = ptr_wrapper_->setTapZoomPoint(static_cast<PayloadIndex>(request.payload_index),request.multiplier, request.x, request.y);
  return response.result;
}

bool VehicleNode::cameraSetZoomParaCallback(CameraSetZoomPara::Request& request, CameraSetZoomPara::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  response.result = ptr_wrapper_->setZoom(static_cast<PayloadIndex>(request.payload_index), request.factor);
  return response.result;
}

bool VehicleNode::cameraZoomCtrlCallback(CameraZoomCtrl::Request& request, CameraZoomCtrl::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }
  if (request.start_stop == 1)
  {
    response.result = ptr_wrapper_->startZoom(static_cast<PayloadIndex>(request.payload_index), request.direction, request.speed);
  }
  if (request.start_stop == 0)
  {
    response.result = ptr_wrapper_->stopZoom(static_cast<PayloadIndex>(request.payload_index));
  }
  return response.result;
}

bool VehicleNode::cameraStartShootSinglePhotoCallback(CameraStartShootSinglePhoto::Request& request, CameraStartShootSinglePhoto::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }
  response.result = ptr_wrapper_->startShootSinglePhoto(static_cast<PayloadIndex>(request.payload_index));
  return response.result;
}

bool VehicleNode::cameraStartShootAEBPhotoCallback(CameraStartShootAEBPhoto::Request& request, CameraStartShootAEBPhoto::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }
  response.result = ptr_wrapper_->startShootAEBPhoto(static_cast<PayloadIndex>(request.payload_index), static_cast<PhotoAEBCount>(request.photo_aeb_count));
  return response.result;
}

bool VehicleNode::cameraStartShootBurstPhotoCallback(CameraStartShootBurstPhoto::Request& request, CameraStartShootBurstPhoto::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }
  response.result = ptr_wrapper_->startShootBurstPhoto(static_cast<PayloadIndex>(request.payload_index),static_cast<PhotoBurstCount>(request.photo_burst_count));
  return response.result;
}

bool VehicleNode::cameraStartShootIntervalPhotoCallback(CameraStartShootIntervalPhoto::Request& request, CameraStartShootIntervalPhoto::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }
  PhotoIntervalData photoIntervalData;
  photoIntervalData.photoNumConticap = request.photo_num_conticap;
  photoIntervalData.timeInterval = request.time_interval;
  response.result = ptr_wrapper_->startShootIntervalPhoto(static_cast<PayloadIndex>(request.payload_index), photoIntervalData);
  return response.result;
}

bool VehicleNode::cameraStopShootPhotoCallback(CameraStopShootPhoto::Request& request, CameraStopShootPhoto::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }
  response.result = ptr_wrapper_->shootPhotoStop(static_cast<PayloadIndex>(request.payload_index));
  return response.result;
}

bool VehicleNode::cameraRecordVideoActionCallback(CameraRecordVideoAction::Request& request, CameraRecordVideoAction::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }
  if(request.start_stop == 1)
  {
    response.result = ptr_wrapper_->startRecordVideo(static_cast<PayloadIndex>(request.payload_index));
  }
  if(request.start_stop == 0)
  {
    response.result = ptr_wrapper_->stopRecordVideo(static_cast<PayloadIndex>(request.payload_index));
  }
  return response.result;
}

bool VehicleNode::getWholeBatteryInfoCallback(GetWholeBatteryInfo::Request& request,GetWholeBatteryInfo::Response& response)
{
  ROS_INFO_STREAM("get Whole Battery Info callback");
  if (ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  DJI::OSDK::BatteryWholeInfo batteryWholeInfo;
  if (ptr_wrapper_->getBatteryWholeInfo(batteryWholeInfo))
  {
    response.battery_whole_info.remainFlyTime  = batteryWholeInfo.remainFlyTime;
    response.battery_whole_info.goHomeNeedTime = batteryWholeInfo.goHomeNeedTime ;
    response.battery_whole_info.landNeedTime   = batteryWholeInfo.landNeedTime;
    response.battery_whole_info.goHomeNeedCapacity = batteryWholeInfo.goHomeNeedCapacity;
    response.battery_whole_info.landNeedCapacity = batteryWholeInfo.landNeedCapacity ;
    response.battery_whole_info.safeFlyRadius = batteryWholeInfo.safeFlyRadius;
    response.battery_whole_info.capacityConsumeSpeed = batteryWholeInfo.capacityConsumeSpeed;
    response.battery_whole_info.goHomeCountDownState = batteryWholeInfo.goHomeCountDownState;
    response.battery_whole_info.gohomeCountDownvalue = batteryWholeInfo.gohomeCountDownvalue;
    response.battery_whole_info.voltage = batteryWholeInfo.voltage;
    response.battery_whole_info.batteryCapacityPercentage = batteryWholeInfo.batteryCapacityPercentage;
    response.battery_whole_info.lowBatteryAlarmThreshold = batteryWholeInfo.lowBatteryAlarmThreshold;
    response.battery_whole_info.lowBatteryAlarmEnable = batteryWholeInfo.lowBatteryAlarmEnable;
    response.battery_whole_info.seriousLowBatteryAlarmThreshold = batteryWholeInfo.seriousLowBatteryAlarmThreshold;
    response.battery_whole_info.seriousLowBatteryAlarmEnable = batteryWholeInfo.seriousLowBatteryAlarmEnable;

    response.battery_whole_info.batteryState.voltageNotSafety        = batteryWholeInfo.batteryState.voltageNotSafety;
    response.battery_whole_info.batteryState.veryLowVoltageAlarm     = batteryWholeInfo.batteryState.veryLowVoltageAlarm;
    response.battery_whole_info.batteryState.LowVoltageAlarm         = batteryWholeInfo.batteryState.LowVoltageAlarm;
    response.battery_whole_info.batteryState.seriousLowCapacityAlarm = batteryWholeInfo.batteryState.seriousLowCapacityAlarm;
    response.battery_whole_info.batteryState.LowCapacityAlarm        = batteryWholeInfo.batteryState.LowCapacityAlarm;
  }
  else
  {
    DSTATUS("get Battery Whole Info failed!");
    return false;
  }
  return true;
}

bool VehicleNode::getSingleBatteryDynamicInfoCallback(GetSingleBatteryDynamicInfo::Request& request, GetSingleBatteryDynamicInfo::Response& response)
{
  //ROS_INFO_STREAM("get Single Battery Dynamic Info callback");
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  DJI::OSDK::SmartBatteryDynamicInfo SmartBatteryDynamicInfo;
  if (ptr_wrapper_->getSingleBatteryDynamicInfo(static_cast<DJI::OSDK::DJIBattery::RequestSmartBatteryIndex>(request.batteryIndex),
                                                SmartBatteryDynamicInfo))
  {
    response.smartBatteryDynamicInfo.batteryIndex           = SmartBatteryDynamicInfo.batteryIndex;
    response.smartBatteryDynamicInfo.currentVoltage         = SmartBatteryDynamicInfo.currentVoltage;
    response.smartBatteryDynamicInfo.currentElectric        = SmartBatteryDynamicInfo.currentElectric;
    response.smartBatteryDynamicInfo.fullCapacity           = SmartBatteryDynamicInfo.fullCapacity;
    response.smartBatteryDynamicInfo.remainedCapacity       = SmartBatteryDynamicInfo.remainedCapacity;
    response.smartBatteryDynamicInfo.batteryTemperature     = SmartBatteryDynamicInfo.batteryTemperature;
    response.smartBatteryDynamicInfo.cellCount              = SmartBatteryDynamicInfo.cellCount;
    response.smartBatteryDynamicInfo.batteryCapacityPercent = SmartBatteryDynamicInfo.batteryCapacityPercent;
    response.smartBatteryDynamicInfo.SOP                    = SmartBatteryDynamicInfo.SOP;

    response.smartBatteryDynamicInfo.batteryState.cellBreak                    = SmartBatteryDynamicInfo.batteryState.cellBreak;
    response.smartBatteryDynamicInfo.batteryState.selfCheckError               = SmartBatteryDynamicInfo.batteryState.selfCheckError;
    response.smartBatteryDynamicInfo.batteryState.batteryClosedReason          = SmartBatteryDynamicInfo.batteryState.batteryClosedReason;
    response.smartBatteryDynamicInfo.batteryState.batSOHState                  = SmartBatteryDynamicInfo.batteryState.batSOHState;
    response.smartBatteryDynamicInfo.batteryState.maxCycleLimit                = SmartBatteryDynamicInfo.batteryState.maxCycleLimit;
    response.smartBatteryDynamicInfo.batteryState.batteryCommunicationAbnormal = SmartBatteryDynamicInfo.batteryState.batteryCommunicationAbnormal;
    response.smartBatteryDynamicInfo.batteryState.hasCellBreak                 = SmartBatteryDynamicInfo.batteryState.hasCellBreak;
    response.smartBatteryDynamicInfo.batteryState.heatState                    = SmartBatteryDynamicInfo.batteryState.heatState;
  }
  else
  {
    DSTATUS("get Single Battery Dynamic Info failed!");
    return false;
  }

  return true;
}

bool VehicleNode::getHMSDataCallback(GetHMSData::Request& request, GetHMSData::Response& response)
{
  //ROS_INFO_STREAM("Get HMS Data callback");
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }
  response.result = true;

  static uint8_t count = 0;
  while (count < 1)
  {
    response.result = ptr_wrapper_->enableSubscribeHMSInfo(request.enable);
    count++;
  }

  if (request.enable == true)
  {
    dji_sdk::HMSPushPacket hmsPushPacket;
    ptr_wrapper_->getHMSListInfo(hmsPushPacket);
    ptr_wrapper_->getHMSDeviceIndex(response.deviceIndex);
    response.timeStamp = hmsPushPacket.timeStamp;

    if (hmsPushPacket.hmsPushData.errList.size())
    {
      response.errList.clear();
      response.errList.resize(hmsPushPacket.hmsPushData.errList.size());
    }
    
    for (int i = 0; i < hmsPushPacket.hmsPushData.errList.size(); i++)
    {
      response.errList[i].alarmID     = hmsPushPacket.hmsPushData.errList[i].alarmID;
      response.errList[i].reportLevel = hmsPushPacket.hmsPushData.errList[i].reportLevel;
      response.errList[i].sensorIndex = hmsPushPacket.hmsPushData.errList[i].sensorIndex;
      // DSTATUS("%ld, response.errList.size():%d,0x%08x,%d,%d",response.timeStamp,response.errList.size(),response.errList[i].alarmID,
      // response.errList[i].sensorIndex,
      // response.errList[i].reportLevel);
    }
  }

  return response.result;
}

bool VehicleNode::mfioCtrlCallback(MFIO::Request& request, MFIO::Response& response)
{
  ROS_INFO_STREAM("MFIO Control callback");
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  if(request.mode == MFIO::Request::MODE_PWM_OUT ||
     request.mode == MFIO::Request::MODE_GPIO_OUT)
  {
    switch (request.action)
    {
      case MFIO::Request::TURN_ON:
        {
          ptr_wrapper_->outputMFIO(request.mode, request.channel, request.init_on_time_us, request.pwm_freq, request.block, request.gpio_value);
        break;
        }
      default:
        {
          response.read_value = ptr_wrapper_->stopMFIO(request.mode, request.channel);
          break;
        }
    }
  }
  else if(request.mode == MFIO::Request::MODE_GPIO_IN ||
          request.mode == MFIO::Request::MODE_ADC)
  {
    response.read_value = ptr_wrapper_->inputMFIO(request.mode, request.channel, request.block);
  }
  return true;
}

bool VehicleNode::setGoHomeAltitudeCallback(SetGoHomeAltitude::Request& request, SetGoHomeAltitude::Response& response)
{
  ROS_INFO_STREAM("Set go home altitude callback");
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }
  if(request.altitude < 5)
  {
    ROS_WARN_STREAM("Altitude for going Home is TOO LOW");
    response.result = false;
    return true;
  }
  if(ptr_wrapper_->setHomeAltitude(request.altitude) == true)
  {
    response.result = true;
  }
  else
  {
    response.result = false;
  }
  return true;
}

bool VehicleNode::getGoHomeAltitudeCallback(GetGoHomeAltitude::Request& request, GetGoHomeAltitude::Response& response)
{
  ROS_INFO_STREAM("Get go home altitude callback");
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  uint16_t altitude = 0;
  if (!(ptr_wrapper_->getHomeAltitude(altitude)))
  {
    response.result = false;
    return false;
  }
  
  response.altitude = altitude;
  response.result = true;
  return true;
}

bool VehicleNode::setCurrentAircraftLocAsHomeCallback(SetCurrentAircraftLocAsHomePoint::Request& request, SetCurrentAircraftLocAsHomePoint::Response& response)
{
  ROS_INFO_STREAM("Set current aircraft location as new home point callback");
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  if(ptr_wrapper_->setCurrentAircraftLocAsHomePoint() == true)
  {
    response.result = true;
  }
  else
  {
    response.result = false;
  }

  return true;
}

bool VehicleNode::setHomePointCallback(SetHomePoint::Request& request, SetHomePoint::Response& response)
{
  ROS_INFO_STREAM("Set home point callback");
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  if (ptr_wrapper_->setHomePoint(request.latitude, request.longitude))
  {
    response.result = true;
    return true;
  }

  response.result = false;
  return false;
}

bool VehicleNode::setLocalPosRefCallback(dji_sdk::SetLocalPosRef::Request &request, dji_sdk::SetLocalPosRef::Response &response)
{
  ROS_INFO("Currrent GPS health is %d",current_gps_health_ );
  if (current_gps_health_ > 3)
  {
    local_pos_ref_latitude_ = current_gps_latitude_;
    local_pos_ref_longitude_ = current_gps_longitude_;
    local_pos_ref_altitude_ = current_gps_altitude_;
    ROS_INFO("Local Position reference has been set.");
    ROS_INFO("MONITOR GPS HEALTH WHEN USING THIS TOPIC");
    local_pos_ref_set_ = true;

    // Create message to publish to a topic
    sensor_msgs::NavSatFix localFrameLLA;
    localFrameLLA.latitude = local_pos_ref_latitude_;
    localFrameLLA.longitude = local_pos_ref_longitude_;
    localFrameLLA.altitude = local_pos_ref_altitude_;
    local_frame_ref_publisher_.publish(localFrameLLA);

    response.result = true;
  }
  else
  {
    ROS_INFO("Not enough GPS Satellites. ");
    ROS_INFO("Cannot set Local Position reference");
    local_pos_ref_set_ = false;
    response.result = false;
  }
  return true;
}

bool VehicleNode::setHorizonAvoidCallback(SetAvoidEnable::Request& request, SetAvoidEnable::Response& response)
{
  ROS_INFO_STREAM("Set horizon avoid function callback");
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  if (!(ptr_wrapper_->setCollisionAvoidance(request.enable)))
  {
    response.result = false;
    return false;
  }

  response.result = true;
  return true;
}

bool VehicleNode::setUpwardsAvoidCallback(SetAvoidEnable::Request& request, SetAvoidEnable::Response& response)
{
  ROS_INFO_STREAM("Set upwards avoid function callback");
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  if(!(ptr_wrapper_->setUpwardsAvoidance(request.enable)))
  {
    response.result = false;
    return false;
  }


  response.result = true;
  return true;
}

bool VehicleNode::getAvoidEnableStatusCallback(GetAvoidEnable::Request& request, GetAvoidEnable::Response& response)
{
  ROS_INFO_STREAM("Set upwards avoid function callback");
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  uint8_t get_horizon_avoid_enable_status = 0xF;
  uint8_t get_upwards_avoid_enable_status = 0xF;

  if (!(ptr_wrapper_->getCollisionAvoidance(get_horizon_avoid_enable_status)))
  {
    response.result = false;
    return false;
  }
  response.horizon_avoid_enable_status = get_horizon_avoid_enable_status;

  if(!(ptr_wrapper_->getUpwardsAvoidance(get_upwards_avoid_enable_status)))
  {
    response.result = false;
    return false;
  }

  response.upwards_avoid_enable_status = get_horizon_avoid_enable_status;

  response.result = true;
  return true;
}

bool VehicleNode::obtainReleaseControlAuthorityCallback(ObtainControlAuthority::Request& request, ObtainControlAuthority::Response& response)
{
  if(request.enable_obtain)
  {
    ROS_INFO_STREAM("Obtain Control Authority Callback");
  }
  else
  {
    ROS_INFO_STREAM("release Control Authority Callback");
  }

  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  response.result = ptr_wrapper_->obtainReleaseCtrlAuthority(request.enable_obtain, FLIGHT_CONTROL_WAIT_TIMEOUT);

  return response.result;
}

bool VehicleNode::killSwitchCallback(KillSwitch::Request& request, KillSwitch::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }
  char msg[10] = "StopFLy";
  response.result = ptr_wrapper_->killSwitch(request.enable, msg);

  return response.result;
}

bool VehicleNode::emergencyBrakeCallback(EmergencyBrake::Request& request, EmergencyBrake::Response& response)
{
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
    return false;
  }

  response.result = ptr_wrapper_->emergencyBrake();

  return response.result;
}

int main(int argc, char** argv)
{
  std::cout << "argc:\t" << argc << "\n";
  for (int i = 0; i < argc; i++){
      std::cout << "\targv[" << i << "]:\t" << argv[i] << "\n";
  }
  std::cout << "\n";

  const char* boardIdFile = "/var/lib/dbus/machine-id";
  std::string boardId;
  ifstream f;
  f.open(boardIdFile, std::ifstream::in);
  std::getline(f, boardId);
  f.close();

  std::string boardIdSubstring = boardId.substr(0, 4);
  std::string vehicleNodeName = "vehicle_node_"+boardIdSubstring;

  std::cout << "vehicleNodeName:\t" << vehicleNodeName << "\n";

  ros::init(argc, argv, vehicleNodeName);
  VehicleNode vh_node;

  ros::spin();
  return 0;
}
