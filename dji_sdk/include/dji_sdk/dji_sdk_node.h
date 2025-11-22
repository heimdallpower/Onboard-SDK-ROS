/** @file dji_sdk_node.h
 *  @version 3.7
 *  @date July, 2018
 *
 *  @brief
 *  A ROS wrapper to interact with DJI onboard SDK
 *
 *  @copyright 2018 DJI. All rights reserved.
 *
 */

#ifndef DJI_SDK_NODE_MAIN_H
#define DJI_SDK_NODE_MAIN_H

// #define COMPARE_PPS_AND_SOFTSYNC

//! ROS
#include <rclcpp/rclcpp.hpp>

//! ROS standard msgs
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <nmea_msgs/msg/sentence.hpp>
#include <tf2_ros/transform_broadcaster.h>

//! msgs
#include <dji_sdk/msg/gimbal.hpp>
#include <dji_sdk/msg/mobile_data.hpp>
#include <dji_sdk/msg/payload_data.hpp>
#include <dji_sdk/msg/flight_anomaly.hpp>
#include <dji_sdk/msg/vo_position.hpp>
#include <dji_sdk/msg/fc_time_in_utc.hpp>
#include <dji_sdk/msg/gpsutc.hpp>
#include <dji_sdk/msg/rtk_yaw.hpp>
#include <dji_sdk/msg/rtk_position.hpp>
#include <dji_sdk/msg/gps_position.hpp>
#include <dji_sdk/msg/gps_health.hpp>
#include <dji_sdk/msg/gps_raw.hpp>
#include <dji_sdk/msg/baro_height.hpp>
#include <dji_sdk/msg/u_int8_stamped.hpp>
#include <dji_sdk/msg/u_int32_stamped.hpp>
#include <dji_sdk/msg/date_time_stamped.hpp>
#include <dji_sdk/msg/int64_stamped.hpp>
#ifdef COMPARE_PPS_AND_SOFTSYNC
#include <dji_sdk/msg/package_timestamp_debug_stamped.hpp>
#include <dji_sdk/msg/hard_sync_debug_stamped.hpp>
#endif
//! mission service
// missionManager
#include <dji_sdk/srv/mission_status.hpp>
// waypoint
#include <dji_sdk/srv/mission_wp_action.hpp>
#include <dji_sdk/srv/mission_wp_get_info.hpp>
#include <dji_sdk/srv/mission_wp_get_speed.hpp>
#include <dji_sdk/srv/mission_wp_set_speed.hpp>
#include <dji_sdk/srv/mission_wp_upload.hpp>
// hotpoint
#include <dji_sdk/srv/mission_hp_action.hpp>
#include <dji_sdk/srv/mission_hp_get_info.hpp>
#include <dji_sdk/srv/mission_hp_reset_yaw.hpp>
#include <dji_sdk/srv/mission_hp_update_radius.hpp>
#include <dji_sdk/srv/mission_hp_update_yaw_rate.hpp>
#include <dji_sdk/srv/mission_hp_upload.hpp>
// hardsync
#include <dji_sdk/srv/set_hard_sync.hpp>

//! service headers
#include <dji_sdk/srv/activation.hpp>
#include <dji_sdk/srv/camera_action.hpp>
#include <dji_sdk/srv/drone_arm_control.hpp>
#include <dji_sdk/srv/drone_task_control.hpp>
#include <dji_sdk/srv/mfio_config.hpp>
#include <dji_sdk/srv/mfio_set_value.hpp>
#include <dji_sdk/srv/sdk_control_authority.hpp>
#include <dji_sdk/srv/set_local_pos_ref.hpp>
#include <dji_sdk/srv/send_mobile_data.hpp>
#include <dji_sdk/srv/send_payload_data.hpp>
#include <dji_sdk/srv/query_drone_version.hpp>
#ifdef ADVANCED_SENSING
#include <dji_sdk/srv/stereo240p_subscription.hpp>
#include <dji_sdk/srv/stereo_depth_subscription.hpp>
#include <dji_sdk/srv/stereo_vga_subscription.hpp>
#include <dji_sdk/srv/setup_camera_stream.hpp>
#endif

//! PPS synchronization
#include "pps_synchronizer.hpp"

//! SDK library
#include <djiosdk/dji_vehicle.hpp>

using namespace DJI::OSDK;

class DJISDKNode: public rclcpp::Node
{
public:
  DJISDKNode(std::string&& name);
  ~DJISDKNode();

  enum TELEMETRY_TYPE
  {
    USE_BROADCAST = 0,
    USE_SUBSCRIBE = 1
  };

  enum
  {
    PACKAGE_ID_5HZ   = 0,
    PACKAGE_ID_50HZ  = 1,
    PACKAGE_ID_100HZ = 2,
    PACKAGE_ID_400HZ = 3
  };

private:
  bool initVehicle(void);
  bool initServices(void);
  bool initFlightControl(void);
  bool initSubscriber(void);
  bool initPublisher(void);
  bool initActions(void);
  bool initDataSubscribeFromFC(void);
  void cleanUpSubscribeFromFC();
  bool validateSerialDevice(LinuxSerialDevice* serialDevice);
  bool isM100();

  /*!
   * @note this function exists here instead of inside the callback function
   *        due to the usages, i.e. we not only provide service call but also
   *        call it for the user when this node was instantiated
   *        we cannot call a service without serviceClient, which is in another
   * node
   */
  ACK::ErrorCode activate(int l_app_id, std::string l_enc_key);

  //! flight control subscriber callbacks
  void flightControlSetpointCallback(const sensor_msgs::msg::Joy::SharedPtr pMsg);
  void flightControlPxPyPzYawCallback(const sensor_msgs::msg::Joy::SharedPtr pMsg);
  void flightControlVxVyVzYawrateCallback(const sensor_msgs::msg::Joy::SharedPtr pMsg);
  void flightControlRollPitchPzYawrateCallback(const sensor_msgs::msg::Joy::SharedPtr pMsg);
  //! general subscriber callbacks
  void gimbalAngleCtrlCallback(const dji_sdk::msg::Gimbal::SharedPtr msg);
  void gimbalSpeedCtrlCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);

  //! general service callbacks
  bool droneActivationCallback(const dji_sdk::srv::Activation::Request::SharedPtr  request,
                               dji_sdk::srv::Activation::Response::SharedPtr response);
  bool sdkCtrlAuthorityCallback(
    const dji_sdk::srv::SDKControlAuthority::Request::SharedPtr  request,
    dji_sdk::srv::SDKControlAuthority::Response::SharedPtr response);
  bool setLocalPosRefCallback(
      const dji_sdk::srv::SetLocalPosRef::Request::SharedPtr  request,
      dji_sdk::srv::SetLocalPosRef::Response::SharedPtr response);
  //! control service callbacks
  bool droneArmCallback(const dji_sdk::srv::DroneArmControl::Request::SharedPtr  request,
                        dji_sdk::srv::DroneArmControl::Response::SharedPtr response);
  bool droneTaskCallback(const dji_sdk::srv::DroneTaskControl::Request::SharedPtr  request,
                         dji_sdk::srv::DroneTaskControl::Response::SharedPtr response);

  //! Mobile Data Service
  bool sendToMobileCallback(const dji_sdk::srv::SendMobileData::Request::SharedPtr  request,
                            dji_sdk::srv::SendMobileData::Response::SharedPtr response);
  //! Payload Data Service
  bool sendToPayloadCallback(dji_sdk::srv::SendPayloadData::Request::SharedPtr request,
                             dji_sdk::srv::SendPayloadData::Response::SharedPtr response);
  //! Query Drone FW version
  bool queryVersionCallback(const dji_sdk::srv::QueryDroneVersion::Request::SharedPtr request,
                            dji_sdk::srv::QueryDroneVersion::Response::SharedPtr response);

  bool cameraActionCallback(const dji_sdk::srv::CameraAction::Request::SharedPtr  request,
                            dji_sdk::srv::CameraAction::Response::SharedPtr response);
  //! mfio service callbacks
  bool MFIOConfigCallback(const dji_sdk::srv::MFIOConfig::Request::SharedPtr  request,
                          dji_sdk::srv::MFIOConfig::Response::SharedPtr response);
  bool MFIOSetValueCallback(const dji_sdk::srv::MFIOSetValue::Request::SharedPtr  request,
                            dji_sdk::srv::MFIOSetValue::Response::SharedPtr response);
  //! mission service callbacks
  // mission manager
  bool missionStatusCallback(const dji_sdk::srv::MissionStatus::Request::SharedPtr  request,
                             dji_sdk::srv::MissionStatus::Response::SharedPtr response);
  // waypoint mission
  bool missionWpUploadCallback(const dji_sdk::srv::MissionWpUpload::Request::SharedPtr  request,
                               dji_sdk::srv::MissionWpUpload::Response::SharedPtr response);
  bool missionWpActionCallback(const dji_sdk::srv::MissionWpAction::Request::SharedPtr  request,
                               dji_sdk::srv::MissionWpAction::Response::SharedPtr response);
  bool missionWpGetInfoCallback(const dji_sdk::srv::MissionWpGetInfo::Request::SharedPtr  request,
                                dji_sdk::srv::MissionWpGetInfo::Response::SharedPtr response);
  bool missionWpGetSpeedCallback(
    const dji_sdk::srv::MissionWpGetSpeed::Request::SharedPtr  request,
    dji_sdk::srv::MissionWpGetSpeed::Response::SharedPtr response);
  bool missionWpSetSpeedCallback(
    const dji_sdk::srv::MissionWpSetSpeed::Request::SharedPtr  request,
    dji_sdk::srv::MissionWpSetSpeed::Response::SharedPtr response);
  // hotpoint mission
  bool missionHpUploadCallback(const dji_sdk::srv::MissionHpUpload::Request::SharedPtr  request,
                               dji_sdk::srv::MissionHpUpload::Response::SharedPtr response);
  bool missionHpActionCallback(const dji_sdk::srv::MissionHpAction::Request::SharedPtr  request,
                               dji_sdk::srv::MissionHpAction::Response::SharedPtr response);
  bool missionHpGetInfoCallback(const dji_sdk::srv::MissionHpGetInfo::Request::SharedPtr  request,
                                dji_sdk::srv::MissionHpGetInfo::Response::SharedPtr response);
  bool missionHpUpdateYawRateCallback(
    const dji_sdk::srv::MissionHpUpdateYawRate::Request::SharedPtr  request,
    dji_sdk::srv::MissionHpUpdateYawRate::Response::SharedPtr response);
  bool missionHpResetYawCallback(
    const dji_sdk::srv::MissionHpResetYaw::Request::SharedPtr  request,
    dji_sdk::srv::MissionHpResetYaw::Response::SharedPtr response);
  bool missionHpUpdateRadiusCallback(
    const dji_sdk::srv::MissionHpUpdateRadius::Request::SharedPtr  request,
    dji_sdk::srv::MissionHpUpdateRadius::Response::SharedPtr response);
  //! hard sync service callback
  bool setHardsyncCallback(const dji_sdk::srv::SetHardSync::Request::SharedPtr  request,
                           dji_sdk::srv::SetHardSync::Response::SharedPtr response);

#ifdef ADVANCED_SENSING
  //! stereo image service callback
  bool stereo240pSubscriptionCallback(const dji_sdk::srv::Stereo240pSubscription::Request::SharedPtr  request,
                                      dji_sdk::srv::Stereo240pSubscription::Response::SharedPtr response);
  bool stereoDepthSubscriptionCallback(const dji_sdk::srv::StereoDepthSubscription::Request::SharedPtr  request,
                                       dji_sdk::srv::StereoDepthSubscription::Response::SharedPtr response);
  bool stereoVGASubscriptionCallback(const dji_sdk::srv::StereoVGASubscription::Request::SharedPtr  request,
                                     dji_sdk::srv::StereoVGASubscription::Response::SharedPtr response);
  bool setupCameraStreamCallback(const dji_sdk::srv::SetupCameraStream::Request::SharedPtr  request,
                                 dji_sdk::srv::SetupCameraStream::Response::SharedPtr response);
#endif

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

#ifdef ADVANCED_SENSING
  static void publish240pStereoImage(Vehicle*            vehicle,
                                     RecvContainer       recvFrame,
                                     DJI::OSDK::UserData userData);

  static void publishVGAStereoImage(Vehicle*            vehicle,
                                    RecvContainer       recvFrame,
                                    DJI::OSDK::UserData userData);

  static void publishMainCameraImage(CameraRGBImage img, void* userData);

  static void publishFPVCameraImage(CameraRGBImage img, void* userData);
#endif

private:
  //! OSDK core
  Vehicle* vehicle;
  //! general service servers
  rclcpp::Service<dji_sdk::srv::Activation>::SharedPtr drone_activation_server;
  rclcpp::Service<dji_sdk::srv::SDKControlAuthority>::SharedPtr sdk_ctrlAuthority_server;
  rclcpp::Service<dji_sdk::srv::CameraAction>::SharedPtr camera_action_server;
  //! flight control service servers
  rclcpp::Service<dji_sdk::srv::DroneArmControl>::SharedPtr drone_arm_server;
  rclcpp::Service<dji_sdk::srv::DroneTaskControl>::SharedPtr drone_task_server;
  //! mfio service servers
  rclcpp::Service<dji_sdk::srv::MFIOConfig>::SharedPtr mfio_config_server;
  rclcpp::Service<dji_sdk::srv::MFIOSetValue>::SharedPtr mfio_set_value_server;
  //! mission service servers
  // mission manager
  rclcpp::Service<dji_sdk::srv::MissionStatus>::SharedPtr mission_status_server;
  // waypoint mission
  rclcpp::Service<dji_sdk::srv::MissionWpUpload>::SharedPtr waypoint_upload_server;
  rclcpp::Service<dji_sdk::srv::MissionWpAction>::SharedPtr waypoint_action_server;
  rclcpp::Service<dji_sdk::srv::MissionWpGetInfo>::SharedPtr waypoint_getInfo_server;
  rclcpp::Service<dji_sdk::srv::MissionWpGetSpeed>::SharedPtr waypoint_getSpeed_server;
  rclcpp::Service<dji_sdk::srv::MissionWpSetSpeed>::SharedPtr waypoint_setSpeed_server;
  // hotpoint mission
  rclcpp::Service<dji_sdk::srv::MissionHpUpload>::SharedPtr hotpoint_upload_server;
  rclcpp::Service<dji_sdk::srv::MissionHpAction>::SharedPtr hotpoint_action_server;
  rclcpp::Service<dji_sdk::srv::MissionHpGetInfo>::SharedPtr hotpoint_getInfo_server;
  rclcpp::Service<dji_sdk::srv::MissionHpUpdateYawRate>::SharedPtr hotpoint_setSpeed_server;
  rclcpp::Service<dji_sdk::srv::MissionHpResetYaw>::SharedPtr hotpoint_resetYaw_server;
  rclcpp::Service<dji_sdk::srv::MissionHpUpdateRadius>::SharedPtr hotpoint_setRadius_server;
  // send data to mobile device
  rclcpp::Service<dji_sdk::srv::SendMobileData>::SharedPtr send_to_mobile_server;
  // send data to payload device
  rclcpp::Service<dji_sdk::srv::SendPayloadData>::SharedPtr send_to_payload_server;
  //! hardsync service
  rclcpp::Service<dji_sdk::srv::SetHardSync>::SharedPtr set_hardsync_server;
  //! Query FW version of FC
  rclcpp::Service<dji_sdk::srv::QueryDroneVersion>::SharedPtr query_version_server;
  //! Set Local position reference
  rclcpp::Service<dji_sdk::srv::SetLocalPosRef>::SharedPtr local_pos_ref_server;

#ifdef ADVANCED_SENSING
  //! stereo image service
  rclcpp::Service<dji_sdk::srv::Stereo240pSubscription>::SharedPtr subscribe_stereo_240p_server;
  rclcpp::Service<dji_sdk::srv::StereoDepthSubscription>::SharedPtr subscribe_stereo_depth_server;
  rclcpp::Service<dji_sdk::srv::StereoVGASubscription>::SharedPtr subscribe_stereo_vga_server;
  rclcpp::Service<dji_sdk::srv::SetupCameraStream>::SharedPtr camera_stream_server;
#endif

  //! flight control subscribers
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr flight_control_sub;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr flight_control_position_yaw_sub;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr flight_control_velocity_yawrate_sub;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr flight_control_rollpitch_yawrate_vertpos_sub;

  //! general subscribers
  rclcpp::Subscription<dji_sdk::msg::Gimbal>::SharedPtr gimbal_angle_cmd_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr gimbal_speed_cmd_subscriber;
  //! telemetry data publisher
  rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr attitude_publisher;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr angularRate_publisher;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr acceleration_publisher;
  rclcpp::Publisher<dji_sdk::msg::BaroHeight>::SharedPtr baro_height_publisher;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_publisher;
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr trigger_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;
  rclcpp::Publisher<dji_sdk::msg::UInt8Stamped>::SharedPtr flight_status_publisher;
  rclcpp::Publisher<dji_sdk::msg::GPSHealth>::SharedPtr gps_health_publisher;
  rclcpp::Publisher<dji_sdk::msg::GPSRaw>::SharedPtr gps_raw_publisher;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_position_publisher;
  rclcpp::Publisher<dji_sdk::msg::VOPosition>::SharedPtr vo_position_publisher;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr height_publisher;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr velocity_publisher;
  rclcpp::Publisher<dji_sdk::msg::MobileData>::SharedPtr from_mobile_data_publisher;
  rclcpp::Publisher<dji_sdk::msg::PayloadData>::SharedPtr from_payload_data_publisher;
  rclcpp::Publisher<dji_sdk::msg::UInt8Stamped>::SharedPtr displaymode_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr rc_publisher;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr rc_connection_status_publisher;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr rtk_position_publisher;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr rtk_velocity_publisher;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr raw_rtk_yaw_publisher;
  rclcpp::Publisher<dji_sdk::msg::RTKYaw>::SharedPtr rtk_yaw_publisher;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr rtk_position_info_publisher;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr rtk_yaw_info_publisher;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr rtk_connection_status_publisher;
  rclcpp::Publisher<dji_sdk::msg::FlightAnomaly>::SharedPtr flight_anomaly_publisher;
  //! Local (GPS) Position Publisher (Publishes local position in ENU frame)
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr local_position_publisher;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr local_frame_ref_publisher;
  rclcpp::Publisher<dji_sdk::msg::GPSPosition>::SharedPtr local_gps_position_publisher;
  rclcpp::Publisher<dji_sdk::msg::DateTimeStamped>::SharedPtr gps_datetime_publisher;
  //! Local RTK Position Publisher (Publishes local RTK position in ENU frame)
  rclcpp::Publisher<dji_sdk::msg::RTKPosition>::SharedPtr local_rtk_position_publisher;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr local_rtk_frame_ref_publisher;
  //! Local RTK/GPS fused position publisher (Publishes high rate local RTK position in ENU frame)
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr local_rtk_fused_position_publisher;
  rclcpp::Publisher<nmea_msgs::msg::Sentence>::SharedPtr time_sync_nmea_publisher;
  rclcpp::Publisher<dji_sdk::msg::GPSUTC>::SharedPtr time_sync_gps_utc_publisher;
  rclcpp::Publisher<dji_sdk::msg::FCTimeInUTC>::SharedPtr time_sync_fc_utc_publisher;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr time_sync_pps_source_publisher;
  //! SDK control authority request ack data publisher
  rclcpp::Publisher<dji_sdk::msg::UInt32Stamped>::SharedPtr control_authority_ack_publisher;

  rclcpp::Publisher<dji_sdk::msg::Int64Stamped>::SharedPtr stamp_diff_5hz_pub;
  rclcpp::Publisher<dji_sdk::msg::Int64Stamped>::SharedPtr stamp_diff_50hz_pub;
  rclcpp::Publisher<dji_sdk::msg::Int64Stamped>::SharedPtr stamp_diff_100hz_pub;
  rclcpp::Publisher<dji_sdk::msg::Int64Stamped>::SharedPtr stamp_diff_400hz_pub;
#ifdef COMPARE_PPS_AND_SOFTSYNC
  rclcpp::Publisher<dji_sdk::msg::HardSyncDebugStamped>::SharedPtr hardsync_debug_publisher;
  rclcpp::Publisher<dji_sdk::msg::PackageTimestampDebugStamped>::SharedPtr packagetimestamp_sub400Hz_debug_publisher;
  rclcpp::Publisher<dji_sdk::msg::PackageTimestampDebugStamped>::SharedPtr packagetimestamp_400Hz_debug_publisher;
  rclcpp::Publisher<dji_sdk::msg::Int64Stamped>::SharedPtr softsync_400hz_lag_pub;
  rclcpp::Publisher<dji_sdk::msg::Int64Stamped>::SharedPtr softsync_sub400hz_lag_pub;
#endif

#ifdef ADVANCED_SENSING
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr stereo_240p_front_left_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr stereo_240p_front_right_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr stereo_240p_down_front_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr stereo_240p_down_back_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr stereo_240p_front_depth_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr stereo_vga_front_left_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr stereo_vga_front_right_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr main_camera_stream_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr fpv_camera_stream_publisher;
#endif
  //! constant
  const int WAIT_TIMEOUT           = 10;
  const int MAX_SUBSCRIBE_PACKAGES = 5;
  const int INVALID_VERSION        = 0;

  //! configurations
  int         app_id;
  std::string enc_key;
  std::string drone_version;
  std::string serial_device;
  int         baud_rate;
  int         app_version;
  std::string app_bundle_id; // reserved
  int         uart_or_usb;
  double      gravity_const;

  //! use broadcast or subscription to get telemetry data
  TELEMETRY_TYPE telemetry_from_fc;
  bool stereo_subscription_success;
  bool stereo_vga_subscription_success;
  bool user_select_broadcast;
  const tf2::Matrix3x3 R_FLU2FRD;
  const tf2::Matrix3x3 R_ENU2NED;

  void flightControl(uint8_t flag, float32_t xSP, float32_t ySP, float32_t zSP, float32_t yawSP);

  enum AlignState
  {
    UNALIGNED,
    ALIGNING,
    ALIGNED
  };

  AlignState curr_align_state;

  static int constexpr STABLE_ALIGNMENT_COUNT = 400;
  static double constexpr TIME_DIFF_CHECK = 0.008;
  static double constexpr TIME_DIFF_ALERT = 0.020;
  static uint8_t constexpr RTK_FIX_THRESHOLD = 40;

  ros::Time base_time;

  bool align_time_with_FC;

  bool local_pos_ref_set;
  bool local_rtk_pos_ref_set;

  void alignRosTimeWithFlightController(ros::Time now_time, uint32_t tick);
  void setUpM100DefaultFreq(uint8_t freq[16]);
  void setUpA3N3DefaultFreq(uint8_t freq[16]);
  std::string controlAuthorityErrorString(const uint32_t error_code);

  double local_pos_ref_latitude, local_pos_ref_longitude, local_pos_ref_altitude;
  double current_gps_latitude, current_gps_longitude, current_gps_altitude;
  int current_gps_health;
  double local_rtk_pos_ref_latitude, local_rtk_pos_ref_longitude, local_rtk_pos_ref_altitude;
  double current_rtk_latitude, current_rtk_longitude, current_rtk_altitude;
  double bias_gps_latitude, bias_gps_longitude, bias_gps_altitude;
  int current_rtk_health;
  bool rtkSupport;

  std::unique_ptr<DJISDK::Synchronizer> pps_sync_;

  enum TimeStampSelect: uint8_t
  {
    PPS_SYNC,
    SOFT_SYNC,
    NO_SYNC
  } timestamp_select;

  bool get400HzTimestamp
  (
    const Telemetry::SyncTimestamp& hardsyncTimeStamp,
    const Telemetry::TimeStamp& packageTimeStamp,
    const ros::Time& now_time,
    ros::Time& data_time_of_measurement_out
  );

  bool getSub400HzTimestamp
  (
    const Telemetry::TimeStamp& packageTimeStamp,
    const ros::Time& now_time,
    ros::Time& data_time_of_measurement_out
  );

};

#endif // DJI_SDK_NODE_MAIN_H
