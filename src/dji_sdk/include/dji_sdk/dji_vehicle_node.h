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

#ifndef __DJI_VEHICLE_NODE_HH__
#define __DJI_VEHICLE_NODE_HH__

// Header include
#include <ros/ros.h>
#include <dji_vehicle.hpp>

#include <dji_sdk/vehicle_wrapper.h>
#include <dji_sdk/common_type.h>

#include <memory>
#include <string>

//! ROS standard msgs
#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/TimeReference.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <nmea_msgs/Sentence.h>

/*! services */
//flight control services
#include <dji_sdk/GetDroneType.h>
#include <dji_sdk/FlightTaskControl.h>
#include <dji_sdk/SetJoystickMode.h>
#include <dji_sdk/JoystickAction.h>
#include <dji_sdk/SetGoHomeAltitude.h>
#include <dji_sdk/GetGoHomeAltitude.h>
#include <dji_sdk/SetHomePoint.h>
#include <dji_sdk/SetCurrentAircraftLocAsHomePoint.h>
#include <dji_sdk/SetLocalPosRef.h>
#include <dji_sdk/SetAvoidEnable.h>
#include <dji_sdk/GetAvoidEnable.h>
#include <dji_sdk/ObtainControlAuthority.h>
#include <dji_sdk/KillSwitch.h>
#include <dji_sdk/EmergencyBrake.h>
//Gimbal control services
#include <dji_sdk/GimbalAction.h>

//Camera control services
#include <dji_sdk/CameraEV.h>
#include <dji_sdk/CameraShutterSpeed.h>
#include <dji_sdk/CameraAperture.h>
#include <dji_sdk/CameraISO.h>
#include <dji_sdk/CameraFocusPoint.h>
#include <dji_sdk/CameraTapZoomPoint.h>
#include <dji_sdk/CameraSetZoomPara.h>
#include <dji_sdk/CameraZoomCtrl.h>
#include <dji_sdk/CameraStartShootBurstPhoto.h>
#include <dji_sdk/CameraStartShootAEBPhoto.h>
#include <dji_sdk/CameraStartShootSinglePhoto.h>
#include <dji_sdk/CameraStartShootIntervalPhoto.h>
#include <dji_sdk/CameraStopShootPhoto.h>
#include <dji_sdk/CameraRecordVideoAction.h>

//HMS services
#include <dji_sdk/GetHMSData.h>
//mfio services
#include <dji_sdk/MFIO.h>
//MOP services
#include <dji_sdk/SendMobileData.h>
#include <dji_sdk/SendPayloadData.h>
//mission services
#include <dji_sdk/MissionStatus.h>
#include <dji_sdk/MissionWpUpload.h>
#include <dji_sdk/MissionWpAction.h>
#include <dji_sdk/MissionWpGetSpeed.h>
#include <dji_sdk/MissionWpSetSpeed.h>
#include <dji_sdk/MissionWpGetInfo.h>
#include <dji_sdk/MissionHpUpload.h>
#include <dji_sdk/MissionHpAction.h>
#include <dji_sdk/MissionHpGetInfo.h>
#include <dji_sdk/MissionHpUpdateYawRate.h>
#include <dji_sdk/MissionHpResetYaw.h>
#include <dji_sdk/MissionHpUpdateRadius.h>
//battery services
#include <dji_sdk/GetWholeBatteryInfo.h>
#include <dji_sdk/GetSingleBatteryDynamicInfo.h>
//poweroff
#include<dji_sdk/OdroidPowerOff.h>

//waypointV2.0 services
#include <dji_sdk/InitWaypointV2Setting.h>
#include <dji_sdk/UploadWaypointV2Mission.h>
#include <dji_sdk/UploadWaypointV2Action.h>
#include <dji_sdk/DownloadWaypointV2Mission.h>
#include <dji_sdk/StartWaypointV2Mission.h>
#include <dji_sdk/StopWaypointV2Mission.h>
#include <dji_sdk/PauseWaypointV2Mission.h>
#include <dji_sdk/ResumeWaypointV2Mission.h>
#include <dji_sdk/GenerateWaypointV2Action.h>
#include <dji_sdk/SetGlobalCruisespeed.h>
#include <dji_sdk/GetGlobalCruisespeed.h>
#include <dji_sdk/SubscribeWaypointV2Event.h>
#include <dji_sdk/SubscribeWaypointV2State.h>

#ifdef ADVANCED_SENSING
#include <dji_sdk/SetupCameraH264.h>
#include <dji_sdk/SetupCameraStream.h>
#include <dji_sdk/Stereo240pSubscription.h>
#include <dji_sdk/StereoDepthSubscription.h>
#include <dji_sdk/StereoVGASubscription.h>
#include <dji_sdk/GetM300StereoParams.h>


//openCV libraries
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>

#include <dji_sdk/kalmancustom.hpp>
#include <dji_sdk/H264Decoder.h>
#include <Eigen/Dense>


#endif

/*! msgs */
#include <dji_sdk/Gimbal.h>
#include <dji_sdk/MobileData.h>
#include <dji_sdk/PayloadData.h>
#include <dji_sdk/FlightAnomaly.h>
#include <dji_sdk/VOPosition.h>
#include <dji_sdk/FCTimeInUTC.h>
#include <dji_sdk/GPSUTC.h>
#include <dji_sdk/telemetry2.h>
#include <dji_sdk/WindData.h>
#include <dji_sdk/BoundingBoxes.h>

#include <dji_sdk/EscData.h>
#include <dji_sdk/ESCStatusIndividual.h>

#include <dji_sdk/ComponentList.h>
#include <dji_sdk/Resolution.h>

//File Manager
#include <dji_sdk/MediaFile.h>
#include <dji_sdk/SDContent.h>

#include <dji_sdk/dji_file_mgr_define.hpp>

//waypointV2.0
#include <dji_sdk/WaypointV2.h>
#include <dji_sdk/WaypointV2Action.h>
#include <dji_sdk/WaypointV2AircraftControlActuator.h>
#include <dji_sdk/WaypointV2AircraftControlActuatorFlying.h>
#include <dji_sdk/WaypointV2AircraftControlActuatorRotateHeading.h>
#include <dji_sdk/WaypointV2AssociateTrigger.h>
#include <dji_sdk/WaypointV2CameraActuator.h>
#include <dji_sdk/WaypointV2CameraActuatorFocusParam.h>
#include <dji_sdk/WaypointV2CameraActuatorFocalLengthParam.h>
#include <dji_sdk/WaypointV2Config.h>
#include <dji_sdk/WaypointV2GimbalActuator.h>
#include <dji_sdk/WaypointV2GimbalActuatorRotationParam.h>
#include <dji_sdk/WaypointV2InitSetting.h>
#include <dji_sdk/WaypointV2IntervalTrigger.h>
#include <dji_sdk/WaypointV2ReachpointTrigger.h>
#include <dji_sdk/WaypointV2SampleReachPointTrigger.h>
#include <dji_sdk/WaypointV2TrajectoryTrigger.h>
#include <dji_sdk/WaypointV2MissionEventPush.h>
#include <dji_sdk/WaypointV2MissionStatePush.h>

#include <dji_sdk/pid.hpp>
#include <thread>
#include <chrono>

#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))
#define RAD2DEG(RAD) ((RAD) * (180.0) / (C_PI))
const int WAIT_TIMEOUT = 10;
const int FLIGHT_CONTROL_WAIT_TIMEOUT = 1;



// Declaration
namespace dji_sdk
{
  using namespace DJI::OSDK;
  using namespace Telemetry;
  using namespace cv;
  using namespace dnn;
  using namespace std;

  // Initialize the parameters
  static vector<string> classes;
  static Net net;

  static vector<string> classes_harpy;
  static Net net_harpy;

  static KalmanFilterCustom kf;
  static bool videoOut;
  static cv::VideoWriter videoWriter;
  static bool streaming;  
  static PID controlPIDx;
  static PID controlPIDy;

  //static Ptr<Tracker> tracker;
 

  class VehicleNode
  {
    public:
      VehicleNode();
      VehicleNode(int test);

      ~VehicleNode();

      bool initGimbalModule();
      bool initCameraModule();
      void initService();
      bool initTopic();
      bool initDataSubscribeFromFC();
      bool cleanUpSubscribeFromFC();


      void nearestBox(float x, float y);
      void trackCar(int x, int y, float xdot, float ydot, int width, int height);

      std::string publishName = "";
      std::string boardId = "";
      std::string boardIdSubstring = "";



      static vector<string> getClasses();
      static Net getNetwork();
      static vector<string> getClassesHarpy();
      static Net getNetworkHarpy();
      static KalmanFilterCustom* getKalmanFilter();
      //static Ptr<Tracker> getTracker();
      //static void setTracker(Ptr<Tracker> newTracker);

      static PID* getPIDx();
      static PID* getPIDy();

      bool isM300();

      //waypoint
      //service clients
      ros::Publisher mission_waypoint_upload_report_publisher_; 
      static void publishWaypointMissionUploadReport(std::string report);

    protected:
      /*! services */
      /*! for general */
      ros::ServiceServer get_drone_type_server_;
      /*! for flight control */
      ros::ServiceServer obtain_releae_control_authority_server_;
      ros::ServiceServer task_control_server_;
      ros::ServiceServer set_joystick_mode_server_;
      ros::ServiceServer joystick_action_server_;
      ros::ServiceServer set_home_altitude_server_;
      ros::ServiceServer get_home_altitude_server_;
      ros::ServiceServer set_home_point_server_;
      ros::ServiceServer set_current_aircraft_point_as_home_server_;
      ros::ServiceServer set_local_pos_reference_server_;
      ros::ServiceServer set_horizon_avoid_enable_server_;
      ros::ServiceServer get_avoid_enable_status_server_;
      ros::ServiceServer set_upwards_avoid_enable_server_;
      ros::ServiceServer kill_switch_server_;
      ros::ServiceServer emergency_brake_action_server_;

      /*! for gimbal */
      ros::ServiceServer gimbal_control_server_;
      /*! for camera */
      ros::ServiceServer camera_control_set_EV_server_;
      ros::ServiceServer camera_control_set_shutter_speed_server_;
      ros::ServiceServer camera_control_set_aperture_server_;
      ros::ServiceServer camera_control_set_ISO_server_;
      ros::ServiceServer camera_control_set_focus_point_server_;
      ros::ServiceServer camera_control_set_tap_zoom_point_server_;
      ros::ServiceServer camera_control_set_zoom_para_server_;
      ros::ServiceServer camera_control_zoom_ctrl_server_;
      ros::ServiceServer camera_control_start_shoot_single_photo_server_;
      ros::ServiceServer camera_control_start_shoot_burst_photo_server_;
      ros::ServiceServer camera_control_start_shoot_AEB_photo_server_;
      ros::ServiceServer camera_control_start_shoot_interval_photo_server_;
      ros::ServiceServer camera_control_stop_shoot_photo_server_;
      ros::ServiceServer camera_control_record_video_action_server_;

      /*! for battery */
      ros::ServiceServer get_single_battery_dynamic_info_server_;
      ros::ServiceServer get_whole_battery_info_server_;
      /*! for hms */
      ros::ServiceServer get_hms_data_server_;
      /*! for mfio */
      ros::ServiceServer mfio_control_server_;
      /*! for mobile device */
      ros::ServiceServer send_data_to_mobile_device_server_;
      ros::ServiceClient send_data_to_mobile_device_client_;
      /*! for payload device */
      ros::ServiceServer send_data_to_payload_device_server_;
      /*! for advanced sensing */
#ifdef ADVANCED_SENSING
      ros::ServiceServer setup_camera_stream_server_;
      ros::ServiceServer track_stop_server_;
      ros::ServiceServer setup_camera_h264_server_;
      ros::ServiceServer subscribe_stereo_240p_server_;
      ros::ServiceServer subscribe_stereo_depth_server_;
      ros::ServiceServer subscribe_stereo_vga_server_;
      //ros::ServiceServer get_m300_stereo_params_server_;
#endif
      /*! for mission */
      ros::ServiceServer waypoint_upload_server_;
      //ros::ServiceServer waypoint_action_server_;
      ros::ServiceServer waypoint_getInfo_server_;
      ros::ServiceServer waypoint_getSpeed_server_;
      ros::ServiceServer waypoint_setSpeed_server_;
      ros::ServiceServer hotpoint_upload_server_;
      ros::ServiceServer hotpoint_action_server_;
      ros::ServiceServer hotpoint_getInfo_server_;
      ros::ServiceServer hotpoint_setSpeed_server_;
      ros::ServiceServer hotpoint_resetYaw_server_;
      ros::ServiceServer hotpoint_setRadius_server_;
      ros::ServiceServer mission_status_server_;

      /*! for waypoint2.0 */
      ros::ServiceServer waypointV2_init_setting_server_;
      ros::ServiceServer waypointV2_upload_mission_server_;
      ros::ServiceServer waypointV2_download_mission_server_;
      ros::ServiceServer waypointV2_upload_action_server_;
      ros::ServiceServer waypointV2_start_mission_server_;
      ros::ServiceServer waypointV2_stop_mission_server_;
      ros::ServiceServer waypointV2_pause_mission_server_;
      ros::ServiceServer waypointV2_resume_mission_server_;
      ros::ServiceServer waypointV2_generate_actions_server_;
      ros::ServiceServer waypointv2_set_global_cruisespeed_server_;
      ros::ServiceServer waypointv2_get_global_cruisespeed_server_;
      ros::ServiceServer waypointV2_generate_polygon_server_;
      ros::ServiceServer waypointv2_subscribe_mission_event_server_;
      ros::ServiceServer waypointv2_subscribe_mission_state_server_;

      /*! publishers */
      //! telemetry data publisher
      ros::Publisher attitude_publisher_;
      ros::Publisher angularRate_publisher_;
      ros::Publisher acceleration_publisher_;
      ros::Publisher battery_state_publisher_;
      ros::Publisher trigger_publisher_;
      ros::Publisher imu_publisher_;
      ros::Publisher flight_status_publisher_;
      ros::Publisher gps_health_publisher_;
      ros::Publisher gps_position_publisher_;
      ros::Publisher telemetry_tanslator_publisher_;
      ros::Publisher wind_data_publisher_;
      ros::Publisher vo_position_publisher_;
      ros::Publisher height_publisher_;
      ros::Publisher velocity_publisher_;
      ros::Publisher from_mobile_data_publisher_;
      ros::Publisher from_payload_data_publisher_;
      ros::Publisher gimbal_angle_publisher_;
      ros::Publisher displaymode_publisher_;
      ros::Publisher rc_publisher_;
      ros::Publisher rc_connection_status_publisher_;
      ros::Publisher rtk_position_publisher_;
      ros::Publisher rtk_velocity_publisher_;
      ros::Publisher rtk_yaw_publisher_;
      ros::Publisher rtk_position_info_publisher_;
      ros::Publisher rtk_yaw_info_publisher_;
      ros::Publisher rtk_connection_status_publisher_;
      ros::Publisher flight_anomaly_publisher_;
      //! Local Position Publisher (Publishes local position in ENU frame)
      ros::Publisher local_position_publisher_;
      ros::Publisher local_frame_ref_publisher_;
      ros::Publisher time_sync_nmea_publisher_;
      ros::Publisher time_sync_gps_utc_publisher_;
      ros::Publisher time_sync_fc_utc_publisher_;
      ros::Publisher time_sync_pps_source_publisher_;

      //advanced sensing
      #ifdef ADVANCED_SENSING
      // 720p
      ros::Publisher main_camera_stream_publisher_;
      cv::Size resolution_1280_720 = cv::Size(1280,720);
      ros::Publisher main_camera_photo_publisher_;
      cv::Size photoResolution = cv::Size(1280,720);

      // 270p
      ros::Publisher main_camera_stream_270p30fps_publisher_;
      ros::Publisher main_camera_stream_270p30fps_resolution_publisher_;
      cv::Size resolution_480_270 = cv::Size(480,270);
      
      //608,342
      ros::Publisher main_camera_stream_resolution_publisher_;
      ros::Publisher main_camera_photo_resolution_publisher_;

      ros::Publisher main_camera_sd_contents_publisher_;

      ros::Publisher main_camera_parameters_publisher_;




      ros::Publisher fpv_camera_stream_publisher_;
      ros::Publisher bounding_boxes_publisher_;
      ros::Subscriber track_car_subscriber_;
      ros::Publisher camera_h264_publisher_;
      ros::Publisher stereo_240p_front_left_publisher_;
      ros::Publisher stereo_240p_front_right_publisher_;
      ros::Publisher stereo_240p_down_front_publisher_;
      ros::Publisher stereo_240p_down_back_publisher_;
      ros::Publisher stereo_240p_front_depth_publisher_;
      ros::Publisher stereo_vga_front_left_publisher_;
      ros::Publisher stereo_vga_front_right_publisher_;

      ros::Publisher stereo_depth_publisher_;
      #endif

      //waypointV2
      ros::Publisher waypointV2_mission_state_publisher_;
      ros::Publisher waypointV2_mission_event_publisher_;

      ros::Subscriber scan_area_response_subscriber;

      //ESC
      ros::Publisher esc_data_publisher_;

      //Camera List
      int cameraListSeq = 0;
      ros::Publisher camera_list_publisher_;

      

    protected:
      /*! for general */
      bool getDroneTypeCallback(dji_sdk::GetDroneType::Request &request,
                                dji_sdk::GetDroneType::Response &response);
      /*! for flight control */
      bool taskCtrlCallback(FlightTaskControl::Request& request, FlightTaskControl::Response& response);
      bool setJoystickModeCallback(SetJoystickMode::Request& request, SetJoystickMode::Response& response);
      bool JoystickActionCallback(JoystickAction::Request& request, JoystickAction::Response& response);
      bool setGoHomeAltitudeCallback(SetGoHomeAltitude::Request& request, SetGoHomeAltitude::Response& response);
      bool getGoHomeAltitudeCallback(GetGoHomeAltitude::Request& request, GetGoHomeAltitude::Response& response);
      bool setCurrentAircraftLocAsHomeCallback(SetCurrentAircraftLocAsHomePoint::Request& request, 
                                               SetCurrentAircraftLocAsHomePoint::Response& response);
      bool setHomePointCallback(SetHomePoint::Request& request, SetHomePoint::Response& response);
      bool setLocalPosRefCallback(dji_sdk::SetLocalPosRef::Request &request,
                                  dji_sdk::SetLocalPosRef::Response &response);
      bool setHorizonAvoidCallback(SetAvoidEnable::Request& request, SetAvoidEnable::Response& response);
      bool setUpwardsAvoidCallback(SetAvoidEnable::Request& request, SetAvoidEnable::Response& response);
      bool getAvoidEnableStatusCallback(GetAvoidEnable::Request& request, GetAvoidEnable::Response& response);
      bool obtainReleaseControlAuthorityCallback(ObtainControlAuthority::Request& request, 
                                                 ObtainControlAuthority::Response& reponse);
      bool killSwitchCallback(KillSwitch::Request& request, KillSwitch::Response& response);
      bool emergencyBrakeCallback(EmergencyBrake::Request& request, EmergencyBrake::Response& response);
      /*! for gimbal control */
      bool gimbalCtrlCallback(GimbalAction::Request& request, GimbalAction::Response& response);
      /*! for camera conrol */
      bool cameraSetEVCallback(CameraEV::Request& request, CameraEV::Response& response);
      bool cameraSetShutterSpeedCallback(CameraShutterSpeed::Request& request, CameraShutterSpeed::Response& response);
      bool cameraSetApertureCallback(CameraAperture::Request& request, CameraAperture::Response& response);
      bool cameraSetISOCallback(CameraISO::Request& request, CameraISO::Response& response);
      bool cameraSetFocusPointCallback(CameraFocusPoint::Request& request, CameraFocusPoint::Response& response);
      bool cameraSetTapZoomPointCallback(CameraTapZoomPoint::Request& request, CameraTapZoomPoint::Response& response);
      bool cameraSetZoomParaCallback(CameraSetZoomPara::Request& request, CameraSetZoomPara::Response& response);
      bool cameraZoomCtrlCallback(CameraZoomCtrl::Request& request, CameraZoomCtrl::Response& response);
      bool cameraStartShootSinglePhotoCallback(CameraStartShootSinglePhoto::Request& request, 
                                               CameraStartShootSinglePhoto::Response& response);
      bool cameraStartShootAEBPhotoCallback(CameraStartShootAEBPhoto::Request& request,
                                            CameraStartShootAEBPhoto::Response& response);
      bool cameraStartShootBurstPhotoCallback(CameraStartShootBurstPhoto::Request& request,
                                              CameraStartShootBurstPhoto::Response& response);
      bool cameraStartShootIntervalPhotoCallback(CameraStartShootIntervalPhoto::Request& request, 
                                                 CameraStartShootIntervalPhoto::Response& response);
      bool cameraStopShootPhotoCallback(CameraStopShootPhoto::Request& request, 
                                        CameraStopShootPhoto::Response& response);
      bool cameraRecordVideoActionCallback(CameraRecordVideoAction::Request& request,
                                           CameraRecordVideoAction::Response& response);
      /*! for battery info */
      bool getWholeBatteryInfoCallback(GetWholeBatteryInfo::Request& request,GetWholeBatteryInfo::Response& reponse);
      bool getSingleBatteryDynamicInfoCallback(GetSingleBatteryDynamicInfo::Request& request,
                                               GetSingleBatteryDynamicInfo::Response& response);
      /*! for hms info */
      bool getHMSDataCallback(GetHMSData::Request& request, GetHMSData::Response& response);
      /*! for mfio conrol */
      bool mfioCtrlCallback(MFIO::Request& request, MFIO::Response& response);
      /*! for mobile device */
      bool sendToMobileCallback(dji_sdk::SendMobileData::Request& request,
                                dji_sdk::SendMobileData::Response& response);
      /*! for payload device */
      bool sendToPayloadCallback(dji_sdk::SendPayloadData::Request& request,
                                 dji_sdk::SendPayloadData::Response& response);
      /*! for advanced sensing conrol */
#ifdef ADVANCED_SENSING
      bool setupCameraStreamCallback(dji_sdk::SetupCameraStream::Request& request,
                                     dji_sdk::SetupCameraStream::Response& response);
      bool setupCameraH264Callback(dji_sdk::SetupCameraH264::Request& request,
                                   dji_sdk::SetupCameraH264::Response& response);
      //! stereo image service callback
      bool stereo240pSubscriptionCallback(dji_sdk::Stereo240pSubscription::Request&  request,
                                          dji_sdk::Stereo240pSubscription::Response& response);
      bool stereoDepthSubscriptionCallback(dji_sdk::StereoDepthSubscription::Request&  request,
                                          dji_sdk::StereoDepthSubscription::Response& response);
      bool stereoVGASubscriptionCallback(dji_sdk::StereoVGASubscription::Request&  request,
                                          dji_sdk::StereoVGASubscription::Response& response);
      //bool getM300StereoParamsCallback(dji_sdk::GetM300StereoParams::Request& request,
       //                                dji_sdk::GetM300StereoParams::Response& response);
      void publishAdvancedSeningData();

      void detectAndTrack(CameraRGBImage rgbImg, void* userData);

      bool stopKalmanTrackCallback(dji_sdk::OdroidPowerOff::Request& request,
                               dji_sdk::OdroidPowerOff::Response& response);

      void stopDetector();


#endif
      /*! for mission service callback*/
      // mission manager
      bool missionStatusCallback(dji_sdk::MissionStatus::Request&  request,
                                 dji_sdk::MissionStatus::Response& response);
      // waypoint mission
      bool missionWpUploadCallback(dji_sdk::MissionWpUpload::Request&  request,
                                    dji_sdk::MissionWpUpload::Response& response);
      bool missionWpActionCallback(dji_sdk::MissionWpAction::Request&  request,
                                   dji_sdk::MissionWpAction::Response& response);
      bool missionWpGetInfoCallback(dji_sdk::MissionWpGetInfo::Request&  request,
                                    dji_sdk::MissionWpGetInfo::Response& response);
      bool missionWpGetSpeedCallback(dji_sdk::MissionWpGetSpeed::Request&  request,
                                     dji_sdk::MissionWpGetSpeed::Response& response);
      bool missionWpSetSpeedCallback(dji_sdk::MissionWpSetSpeed::Request&  request,
                                     dji_sdk::MissionWpSetSpeed::Response& response);
      // hotpoint mission
      bool missionHpUploadCallback(dji_sdk::MissionHpUpload::Request&  request,
                                   dji_sdk::MissionHpUpload::Response& response);
      bool missionHpActionCallback(dji_sdk::MissionHpAction::Request&  request,
                                   dji_sdk::MissionHpAction::Response& response);
      bool missionHpGetInfoCallback(dji_sdk::MissionHpGetInfo::Request&  request,
                                    dji_sdk::MissionHpGetInfo::Response& response);
      bool missionHpUpdateYawRateCallback(dji_sdk::MissionHpUpdateYawRate::Request&  request,
                                          dji_sdk::MissionHpUpdateYawRate::Response& response);
      bool missionHpResetYawCallback(dji_sdk::MissionHpResetYaw::Request&  request,
                                     dji_sdk::MissionHpResetYaw::Response& response);
      bool missionHpUpdateRadiusCallback(dji_sdk::MissionHpUpdateRadius::Request&  request,
                                         dji_sdk::MissionHpUpdateRadius::Response& response);
      /*! for waypiontV2.0 service callback*/
      bool waypointV2InitSettingCallback(dji_sdk::InitWaypointV2Setting::Request&  request,
                                         dji_sdk::InitWaypointV2Setting::Response& response);
      bool waypointV2UploadMissionCallback(dji_sdk::UploadWaypointV2Mission::Request&  request,
                                           dji_sdk::UploadWaypointV2Mission::Response& response);
      bool waypointV2DownloadMissionCallback(dji_sdk::DownloadWaypointV2Mission::Request&  request,
                                             dji_sdk::DownloadWaypointV2Mission::Response& response);
      bool waypointV2UploadActionCallback(dji_sdk::UploadWaypointV2Action::Request&  request,
                                          dji_sdk::UploadWaypointV2Action::Response& response);
      bool waypointV2StartMissionCallback(dji_sdk::StartWaypointV2Mission::Request&  request,
                                          dji_sdk::StartWaypointV2Mission::Response& response);
      bool waypointV2StopMissionCallback(dji_sdk::StopWaypointV2Mission::Request&  request,
                                         dji_sdk::StopWaypointV2Mission::Response& response);
      bool waypointV2PauseMissionCallback(dji_sdk::PauseWaypointV2Mission::Request&  request,
                                          dji_sdk::PauseWaypointV2Mission::Response& response);
      bool waypointV2ResumeMissionCallback(dji_sdk::ResumeWaypointV2Mission::Request&  request,
                                           dji_sdk::ResumeWaypointV2Mission::Response& response);
      bool waypointV2GenerateActionsCallback(dji_sdk::GenerateWaypointV2Action::Request&  request,
                                             dji_sdk::GenerateWaypointV2Action::Response& response);
      bool waypointV2SetGlobalCruisespeedCallback(dji_sdk::SetGlobalCruisespeed::Request& request,
                                                  dji_sdk::SetGlobalCruisespeed::Response& respons);
      bool waypointV2GetGlobalCruisespeedCallback(dji_sdk::GetGlobalCruisespeed::Request& request,
                                                  dji_sdk::GetGlobalCruisespeed::Response& response);
      bool waypointV2SubscribeMissionEventCallback(dji_sdk::SubscribeWaypointV2Event::Request& request,
                                                   dji_sdk::SubscribeWaypointV2Event::Response& response);
      bool waypointV2SubscribeMissionStateCallback(dji_sdk::SubscribeWaypointV2State::Request& request,
                                                   dji_sdk::SubscribeWaypointV2State::Response& response);

      bool initSubscribe();

    private:
      ros::NodeHandle nh_;
      VehicleWrapper* ptr_wrapper_;
      TelemetryType telemetry_from_fc_;

      int           app_id_;
      int           app_version_;
      int           baud_rate_;
      double        gravity_const_;
      std::string   enc_key_;
      std::string   device_acm_;
      std::string   device_;
      std::string   sample_case_;
      std::string   drone_version_;
      std::string   app_bundle_id_; // reserved
      bool          user_select_broadcast_;
      bool          align_time_with_FC_;

      AlignStatus curr_align_state_;
      ros::Time   base_time_;
      double      local_pos_ref_latitude_, local_pos_ref_longitude_, local_pos_ref_altitude_;
      double      current_gps_latitude_, current_gps_longitude_, current_gps_altitude_;
      bool        local_pos_ref_set_;
      int         current_gps_health_;
      const       tf::Matrix3x3 R_FLU2FRD_;
      const       tf::Matrix3x3 R_ENU2NED_;
      bool        rtk_support_;

      bool stereo_subscription_success;
      bool stereo_vga_subscription_success;

      std::vector<DJIWaypointV2Action> actions;

      std::thread controlThread;
      std::thread publishMainCameraThread;
      void sendFlightCommands();

    void scanAreaResponseCallback(const dji_sdk::MissionWaypointTask::ConstPtr& waypoints);
    void trackTargetCallback(const std_msgs::Float32MultiArray::ConstPtr& box);

    //! data broadcast callback
    void dataBroadcastCallback();
    void fromMobileDataCallback(RecvContainer recvFrame);

    void fromPayloadDataCallback(RecvContainer recvFrame);

    static void NMEACallback(Vehicle* vehiclePtr,
                             RecvContainer recvFrame,
                             UserData userData);

    static void GPSUTCTimeCallback(Vehicle *vehiclePtr,
                                   RecvContainer recvFrame,
                                   UserData userData);


    static void FCTimeInUTCCallback(Vehicle* vehiclePtr,
                                    RecvContainer recvFrame,
                                    UserData userData);

    static void PPSSourceCallback(Vehicle* vehiclePtr,
                                  RecvContainer recvFrame,
                                  UserData userData);
    static void SDKfromMobileDataCallback(Vehicle*            vehicle,
                                          RecvContainer       recvFrame,
                                          DJI::OSDK::UserData userData);

    static void SDKfromPayloadDataCallback(Vehicle *vehicle,
                                           RecvContainer recvFrame,
                                           DJI::OSDK::UserData userData);

    static void SDKBroadcastCallback(Vehicle*            vehicle,
                                     RecvContainer       recvFrame,
                                     DJI::OSDK::UserData userData);

    static void publish1HzData(Vehicle*            vehicle,
                              RecvContainer       recvFrame,
                              DJI::OSDK::UserData userData);

    static void publish5HzData(Vehicle*            vehicle,
                               RecvContainer       recvFrame,
                               DJI::OSDK::UserData userData);

    static void publish50HzData(Vehicle*            vehicle,
                                RecvContainer       recvFrame,
                                DJI::OSDK::UserData userData);

    static void publish100HzData(Vehicle*            vehicle,
                                 RecvContainer       recvFrame,
                                 DJI::OSDK::UserData userData);

    static void publish400HzData(Vehicle*            vehicle,
                                 RecvContainer       recvFrame,
                                 DJI::OSDK::UserData userData);

    // File Manager
    void requestStorageDeviceFiles(DJI::OSDK::PayloadIndexType PayloadIndex);
    static void fileMgrFileListCb(E_OsdkStat retCode, const FilePackage fileList, void* userData);

#ifdef ADVANCED_SENSING
    static void onClickCallback(int event, int x, int y, int flags, void* param);
    static void publishMainCameraImage(CameraRGBImage rgbImg, void* userData);
    void publishPullMainCameraImage();
    static void publishFPVCameraImage(CameraRGBImage rgbImg, void* userData);
    static void publishCameraH264(uint8_t* buf, int bufLen, void* userData);
    static void publish240pStereoImage(Vehicle*            vehicle,
                                       RecvContainer       recvFrame,
                                       DJI::OSDK::UserData userData);


    static void PerceptionImageCB(Perception::ImageInfoType info, uint8_t *imageRawBuffer,
                       int bufferLen, void *userData);

    static void publishVGAStereoImage(Vehicle*            vehicle,
                                      RecvContainer       recvFrame,
                                      DJI::OSDK::UserData userData);

    float getZoomFactor(Vehicle* vehicle, PayloadIndexType index, const char *name);

    std::vector<uchar> mat2Imgarray(Mat im_tab);
#endif

    static E_OsdkStat updateMissionEvent(T_CmdHandle *cmdHandle, const T_CmdInfo *cmdInfo,
                                         const uint8_t *cmdData, void *userData);
    static E_OsdkStat updateMissionState(T_CmdHandle *cmdHandle, const T_CmdInfo *cmdInfo,
                                         const uint8_t *cmdData, void *userData);

public:
    void gpsConvertENU(double &ENU_x, double &ENU_y,
                       double gps_t_lon, double gps_t_lat,
                       double gps_r_lon, double gps_r_lat);
    void alignRosTimeWithFlightController(ros::Time now_time, uint32_t tick);

  };

  typedef struct VehicleStereoImagePacketType {
    StereoImagePacketType* stereoImagePacketType;
    VehicleNode *vh_node;
  } VehicleStereoImagePacketType;
}

#endif // __DJI_VEHICLE_NODE_HH__

