/** @file dji_sdk_node.cpp
 *  @version 3.3
 *  @date May, 2017
 *
 *  @brief
 *  Implementation of the initialization functions of DJISDKNode
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#include <dji_sdk/dji_sdk_node.h>

using namespace DJI::OSDK;

DJISDKNode::DJISDKNode(std::string&& name):
Node{name},
telemetry_from_fc(USE_BROADCAST),
R_FLU2FRD(tf2::Matrix3x3(1,  0,  0, 0, -1,  0, 0,  0, -1)),
R_ENU2NED(tf2::Matrix3x3(0,  1,  0, 1,  0,  0, 0,  0, -1)),
curr_align_state(UNALIGNED)
{
  declare_parameter("serial_name", std::string("/dev/ttyUSB0"));
  declare_parameter("baud_rate", 921600);
  declare_parameter("app_id", 123456);
  declare_parameter("app_version", 1);
  declare_parameter("enc_key", std::string("abcd1234"));
  declare_parameter("drone_version", std::string("M100")); // choose M100 as default
  declare_parameter("gravity_const", 9.801);
  declare_parameter("software_time_alignment", false);
  declare_parameter("use_broadcast", false);

  serial_device = get_parameter("serial_name").as_string();
  baud_rate = get_parameter("baud_rate").as_int();
  app_id = get_parameter("app_id").as_int();
  app_version = get_parameter("app_version").as_int();
  enc_key = get_parameter("enc_key").as_string();
  drone_version = get_parameter("drone_version").as_string();
  gravity_const = get_parameter("gravity_const").as_double();
  align_time_with_FC = get_parameter("software_time_alignment").as_bool();
  user_select_broadcast = get_parameter("use_broadcast").as_bool();

  //! Default values for local Position
  local_pos_ref_latitude  = 0;
  local_pos_ref_longitude = 0;
  local_pos_ref_altitude  = 0;
  local_pos_ref_set       = false;
  //! Defualt values for local RTK position
  local_rtk_pos_ref_latitude  = 0;
  local_rtk_pos_ref_longitude = 0;
  local_rtk_pos_ref_altitude  = 0;
  local_rtk_pos_ref_set       = false;

  //! Initial values for GPS biases
  bias_gps_latitude   = 0;
  bias_gps_longitude  = 0;
  bias_gps_altitude   = 0;

  //! RTK support check
  rtkSupport = false;

  // @todo need some error handling for init functions
  //! @note parsing launch file to get environment parameters
  if (!initVehicle())
  {
    RCLCPP_ERROR(get_logger(), "Vehicle initialization failed");
    rclcpp::shutdown();
    return;
  }

  std::string pps_device_path;
  declare_parameter("pps_device", rclcpp::ParameterType::PARAMETER_STRING);
  if (!get_parameter("pps_device", pps_device_path))
  {
    RCLCPP_FATAL(get_logger(), "[dji_sdk] PPS device path not supplied. Shutting down.");
    rclcpp::shutdown();
    return;
  }
  double pps_window_half_width_sec;
  declare_parameter("pps_window_half_width_sec", rclcpp::ParameterType::PARAMETER_DOUBLE);
  if (!get_parameter("pps_window_half_width_sec", pps_window_half_width_sec) || (pps_window_half_width_sec >= 0.5))
  {
    RCLCPP_FATAL(get_logger(), "[dji_sdk] PPS window width not supplied/invalid. Shutting down.");
    rclcpp::shutdown();
    return;
  }

  if (pps_device_path != "")
  {
    vehicle->hardSync->setSyncFreq(1ul);

    pps::Handler::CreationStatus pps_creation_status{pps::Handler::CreationStatus::OK};
    pps_sync_ = std::unique_ptr<DJISDK::Synchronizer>(new DJISDK::Synchronizer{
      shared_from_this(),
      pps_device_path,
      pps_window_half_width_sec,
      pps_creation_status
    });
    if (pps_creation_status != pps::Handler::CreationStatus::OK)
    {
      RCLCPP_ERROR_STREAM(get_logger(), "[dji_sdk] PPS init error " << pps_creation_status << ". Shutting down.");
      rclcpp::shutdown();
      return;
    }
    timestamp_select = PPS_SYNC;
    RCLCPP_INFO(get_logger(), "[dji_sdk] PPS used for time synchronization.");
  }
  else if (align_time_with_FC)
  {
    timestamp_select = SOFT_SYNC;
    RCLCPP_INFO(get_logger(), "[dji_sdk] Software used for time synchronization.");
  }
  else
  {
    timestamp_select = NO_SYNC;
    RCLCPP_INFO(get_logger(), "[dji_sdk] No time synchronization. ros::Time::now() of arrival used to stamp data.");
  }

  if (!initServices())
  {
    RCLCPP_ERROR(get_logger(), "initServices failed");
    rclcpp::shutdown();
  }

  if (!initFlightControl())
  {
    RCLCPP_ERROR(get_logger(), "initFlightControl failed");
    rclcpp::shutdown();
  }

  if (!initSubscriber())
  {
    RCLCPP_ERROR(get_logger(), "initSubscriber failed");
    rclcpp::shutdown();
  }

  if (!initPublisher())
  {
    RCLCPP_ERROR(get_logger(), "initPublisher failed");
    rclcpp::shutdown();
  }
}

DJISDKNode::~DJISDKNode()
{
  if(!isM100())
  {
    cleanUpSubscribeFromFC();
  }
  if (vehicle)
  {
    delete vehicle;
  }
}

bool
DJISDKNode::initVehicle(void)
{
  bool threadSupport = true;
  bool enable_advanced_sensing = false;

#ifdef ADVANCED_SENSING
  enable_advanced_sensing = true;
  RCLCPP_INFO(get_logger(), "Advanced Sensing is Enabled on M210.");
#endif

  //! @note currently does not work without thread support
  vehicle = new Vehicle(serial_device.c_str(), baud_rate, threadSupport, enable_advanced_sensing);

  /*!
   * @note activate the drone for the user at the beginning
   *        user can also call it as a service
   *        this has been tested by giving wrong appID in launch file
   */
  if (ACK::getError(this->activate(this->app_id, this->enc_key)))
  {
    RCLCPP_ERROR(get_logger(), "drone activation error");
    return false;
  }
  RCLCPP_INFO(get_logger(), "drone activated");

  // This version of ROS Node works for:
  //    1. A3/N3/M600 with latest FW
  //    2. M100 with FW version M100_31
  if(vehicle->getFwVersion() > INVALID_VERSION
      && vehicle->getFwVersion() < mandatoryVersionBase
      && (!isM100()))
  {
    return false;
  }


  if (NULL != vehicle->subscribe && (!user_select_broadcast))
  {
    telemetry_from_fc = USE_SUBSCRIBE;
  }

  return true;
}

// clang-format off
bool DJISDKNode::initServices(void) {
  // Common to A3/N3 and M100
  using namespace std::placeholders;

  drone_activation_server   = create_service<dji_sdk::srv::Activation>("dji_sdk/activation",                     std::bind(&DJISDKNode::droneActivationCallback, this, _1, _2));
  drone_arm_server          = create_service<dji_sdk::srv::DroneArmControl>("dji_sdk/drone_arm_control",              std::bind(&DJISDKNode::droneArmCallback, this, _1, _2));
  drone_task_server         = create_service<dji_sdk::srv::DroneTaskControl>("dji_sdk/drone_task_control",             std::bind(&DJISDKNode::droneTaskCallback, this, _1, _2));
  sdk_ctrlAuthority_server  = create_service<dji_sdk::srv::SDKControlAuthority>("dji_sdk/sdk_control_authority",          std::bind(&DJISDKNode::sdkCtrlAuthorityCallback, this, _1, _2));
  camera_action_server      = create_service<dji_sdk::srv::CameraAction>("dji_sdk/camera_action",                  std::bind(&DJISDKNode::cameraActionCallback, this, _1, _2));
  waypoint_upload_server    = create_service<dji_sdk::srv::MissionWpUpload>("dji_sdk/mission_waypoint_upload",        std::bind(&DJISDKNode::missionWpUploadCallback, this, _1, _2));
  waypoint_action_server    = create_service<dji_sdk::srv::MissionWpAction>("dji_sdk/mission_waypoint_action",        std::bind(&DJISDKNode::missionWpActionCallback, this, _1, _2));
  waypoint_getInfo_server   = create_service<dji_sdk::srv::MissionWpGetInfo>("dji_sdk/mission_waypoint_getInfo",       std::bind(&DJISDKNode::missionWpGetInfoCallback, this, _1, _2));
  waypoint_getSpeed_server  = create_service<dji_sdk::srv::MissionWpGetSpeed>("dji_sdk/mission_waypoint_getSpeed",      std::bind(&DJISDKNode::missionWpGetSpeedCallback, this, _1, _2));
  waypoint_setSpeed_server  = create_service<dji_sdk::srv::MissionWpSetSpeed>("dji_sdk/mission_waypoint_setSpeed",      std::bind(&DJISDKNode::missionWpSetSpeedCallback, this, _1, _2));
  hotpoint_upload_server    = create_service<dji_sdk::srv::MissionHpUpload>("dji_sdk/mission_hotpoint_upload",        std::bind(&DJISDKNode::missionHpUploadCallback, this, _1, _2));
  hotpoint_action_server    = create_service<dji_sdk::srv::MissionHpAction>("dji_sdk/mission_hotpoint_action",        std::bind(&DJISDKNode::missionHpActionCallback, this, _1, _2));
  hotpoint_getInfo_server   = create_service<dji_sdk::srv::MissionHpGetInfo>("dji_sdk/mission_hotpoint_getInfo",       std::bind(&DJISDKNode::missionHpGetInfoCallback, this, _1, _2));
  hotpoint_setSpeed_server  = create_service<dji_sdk::srv::MissionHpUpdateYawRate>("dji_sdk/mission_hotpoint_updateYawRate", std::bind(&DJISDKNode::missionHpUpdateYawRateCallback, this, _1, _2));
  hotpoint_resetYaw_server  = create_service<dji_sdk::srv::MissionHpResetYaw>("dji_sdk/mission_hotpoint_resetYaw",      std::bind(&DJISDKNode::missionHpResetYawCallback, this, _1, _2));
  hotpoint_setRadius_server = create_service<dji_sdk::srv::MissionHpUpdateRadius>("dji_sdk/mission_hotpoint_updateRadius",  std::bind(&DJISDKNode::missionHpUpdateRadiusCallback, this, _1, _2));
  mission_status_server     = create_service<dji_sdk::srv::MissionStatus>("dji_sdk/mission_status",                 std::bind(&DJISDKNode::missionStatusCallback, this, _1, _2));
  send_to_mobile_server     = create_service<dji_sdk::srv::SendMobileData>("dji_sdk/send_data_to_mobile",            std::bind(&DJISDKNode::sendToMobileCallback, this, _1, _2));
  send_to_payload_server    = create_service<dji_sdk::srv::SendPayloadData>("dji_sdk/send_data_to_payload",           std::bind(&DJISDKNode::sendToPayloadCallback, this, _1, _2));
  query_version_server      = create_service<dji_sdk::srv::QueryDroneVersion>("dji_sdk/query_drone_version",            std::bind(&DJISDKNode::queryVersionCallback, this, _1, _2));
  local_pos_ref_server      = create_service<dji_sdk::srv::SetLocalPosRef>("dji_sdk/set_local_pos_ref",              std::bind(&DJISDKNode::setLocalPosRefCallback, this, _1, _2));
#ifdef ADVANCED_SENSING
  subscribe_stereo_240p_server  = create_service<dji_sdk::srv::Stereo240pSubscription>("dji_sdk/stereo_240p_subscription",   std::bind(&DJISDKNode::stereo240pSubscriptionCallback, this, _1, _2));
  subscribe_stereo_depth_server = create_service<dji_sdk::srv::StereoDepthSubscription>("dji_sdk/stereo_depth_subscription",  std::bind(&DJISDKNode::stereoDepthSubscriptionCallback, this, _1, _2));
  subscribe_stereo_vga_server   = create_service<dji_sdk::srv::StereoVGASubscription>("dji_sdk/stereo_vga_subscription",    std::bind(&DJISDKNode::stereoVGASubscriptionCallback, this, _1, _2));
  camera_stream_server          = create_service<dji_sdk::srv::SetupCameraStream>("dji_sdk/setup_camera_stream",        std::bind(&DJISDKNode::setupCameraStreamCallback, this, _1, _2));
#endif

  // A3/N3 only
  if(!isM100())
  {
    set_hardsync_server   = create_service<dji_sdk::srv::SetHardSync>("dji_sdk/set_hardsyc", std::bind(&DJISDKNode::setHardsyncCallback, this, _1, _2));
    mfio_config_server    = create_service<dji_sdk::srv::MFIOConfig>("dji_sdk/mfio_config", std::bind(&DJISDKNode::MFIOConfigCallback, this, _1, _2));
    mfio_set_value_server = create_service<dji_sdk::srv::MFIOSetValue>("dji_sdk/mfio_set_value", std::bind(&DJISDKNode::MFIOSetValueCallback, this, _1, _2));
  }
  return true;
}
// clang-format on

bool
DJISDKNode::initFlightControl(void)
{
  using namespace std::placeholders;

  flight_control_sub = create_subscription<sensor_msgs::msg::Joy>(
    "dji_sdk/flight_control_setpoint_generic", 10,
    std::bind(&DJISDKNode::flightControlSetpointCallback, this, _1));

  flight_control_position_yaw_sub =
    create_subscription<sensor_msgs::msg::Joy>(
      "dji_sdk/flight_control_setpoint_ENUposition_yaw", 10,
      std::bind(&DJISDKNode::flightControlPxPyPzYawCallback, this, _1));

  flight_control_velocity_yawrate_sub =
    create_subscription<sensor_msgs::msg::Joy>(
      "dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 10,
      std::bind(&DJISDKNode::flightControlVxVyVzYawrateCallback, this, _1));

  flight_control_rollpitch_yawrate_vertpos_sub =
    create_subscription<sensor_msgs::msg::Joy>(
      "dji_sdk/flight_control_setpoint_rollpitch_yawrate_zposition", 10,
      std::bind(&DJISDKNode::flightControlRollPitchPzYawrateCallback, this, _1));

  return true;
}

bool DJISDKNode::isM100()
{
  return(vehicle->isM100());
}


ACK::ErrorCode
DJISDKNode::activate(int l_app_id, std::string l_enc_key)
{
  usleep(1000000);
  Vehicle::ActivateData testActivateData;
  char                  app_key[65];
  testActivateData.encKey = app_key;
  strcpy(testActivateData.encKey, l_enc_key.c_str());
  testActivateData.ID = l_app_id;

  RCLCPP_DEBUG(get_logger(), "called vehicle->activate(&testActivateData, WAIT_TIMEOUT)");
  return vehicle->activate(&testActivateData, WAIT_TIMEOUT);
}

bool
DJISDKNode::initSubscriber(void)
{
  using namespace std::placeholders;

  gimbal_angle_cmd_subscriber = create_subscription<dji_sdk::msg::Gimbal>(
    "dji_sdk/gimbal_angle_cmd", 10, std::bind(&DJISDKNode::gimbalAngleCtrlCallback, this, _1));
  gimbal_speed_cmd_subscriber = create_subscription<geometry_msgs::msg::Vector3Stamped>(
    "dji_sdk/gimbal_speed_cmd", 10, std::bind(&DJISDKNode::gimbalSpeedCtrlCallback, this, _1));
  return true;
}

bool
DJISDKNode::initPublisher(void)
{
  rc_publisher = create_publisher<sensor_msgs::msg::Joy>("dji_sdk/rc", 10);

  attitude_publisher =
    create_publisher<geometry_msgs::msg::QuaternionStamped>("dji_sdk/attitude", 10);

  battery_state_publisher =
    create_publisher<sensor_msgs::msg::BatteryState>("dji_sdk/battery_state",10);

  /*!
   * - Fused attitude (duplicated from attitude topic)
   * - Raw linear acceleration (body frame: FLU, m/s^2)
   *       Z value is +9.8 when placed on level ground statically
   * - Raw angular velocity (body frame: FLU, rad/s^2)
   */
  imu_publisher = create_publisher<sensor_msgs::msg::Imu>("dji_sdk/imu", 10);

  // Refer to dji_sdk.h for different enums for M100 and A3/N3
  flight_status_publisher =
    create_publisher<dji_sdk::msg::UInt8Stamped>("dji_sdk/flight_status", 10);

  /*!
   * gps_health needs to be greater than 3 for gps_position and velocity topics
   * to be trusted
   */
  gps_health_publisher =
    create_publisher<dji_sdk::msg::GPSHealth>("dji_sdk/gps_health", 10);

  gps_raw_publisher =
    create_publisher<dji_sdk::msg::GPSRaw>("dji_sdk/gps_raw", 10);

  /*!
   * NavSatFix specs:
   *   Latitude [degrees]. Positive is north of equator; negative is south.
   *   Longitude [degrees]. Positive is east of prime meridian; negative is
   * west.
   *   Altitude [m]. Positive is above the WGS 84 ellipsoid
   */
  gps_position_publisher =
    create_publisher<sensor_msgs::msg::NavSatFix>("dji_sdk/gps_position", 10);

  /*!
   *   x [m]. Positive along navigation frame x axis
   *   y [m]. Positive along navigation frame y axis
   *   z [m]. Positive is down
   *   For details about navigation frame, please see telemetry documentation in API reference
  */
  vo_position_publisher =
          create_publisher<dji_sdk::msg::VOPosition>("dji_sdk/vo_position", 10);
  /*!
   * Height above home altitude. It is valid only after drone
   * is armed.
   */
  height_publisher =
    create_publisher<std_msgs::msg::Float32>("dji_sdk/height_above_takeoff", 10);

  velocity_publisher =
    create_publisher<geometry_msgs::msg::Vector3Stamped>("dji_sdk/velocity", 10);

  from_mobile_data_publisher =
    create_publisher<dji_sdk::msg::MobileData>("dji_sdk/from_mobile_data", 10);

  from_payload_data_publisher =
    create_publisher<dji_sdk::msg::PayloadData>("dji_sdk/from_payload_data", 10);

  local_position_publisher =
      create_publisher<geometry_msgs::msg::PointStamped>("dji_sdk/local_position", 10);

  local_gps_position_publisher =
      create_publisher<dji_sdk::msg::GPSPosition>("dji_sdk/local_gps_position", 10);

  gps_datetime_publisher =
      create_publisher<dji_sdk::msg::DateTimeStamped>("dji_sdk/gps_datetime", 10);

  local_frame_ref_publisher =
      create_publisher<sensor_msgs::msg::NavSatFix>("dji_sdk/local_frame_ref", rclcpp::QoS(10).transient_local());

  local_rtk_position_publisher =
      create_publisher<dji_sdk::msg::RTKPosition>("dji_sdk/local_rtk_position", 10);

  local_rtk_frame_ref_publisher =
      create_publisher<sensor_msgs::msg::NavSatFix>("dji_sdk/local_rtk_frame_ref", rclcpp::QoS(10).transient_local());

  local_rtk_fused_position_publisher =
      create_publisher<geometry_msgs::msg::PointStamped>("dji_sdk/local_rtk_fused_position", 10);

  time_sync_nmea_publisher =
      create_publisher<nmea_msgs::msg::Sentence>("dji_sdk/time_sync_nmea_msg", 10);

  time_sync_gps_utc_publisher =
      create_publisher<dji_sdk::msg::GPSUTC>("dji_sdk/time_sync_gps_utc", 10);

  time_sync_fc_utc_publisher =
      create_publisher<dji_sdk::msg::FCTimeInUTC>("dji_sdk/time_sync_fc_time_utc", 10);

  time_sync_pps_source_publisher =
      create_publisher<std_msgs::msg::String>("dji_sdk/time_sync_pps_source", 10);

  stamp_diff_5hz_pub =
    create_publisher<dji_sdk::msg::Int64Stamped>("dji_sdk/stamp_diff/5hz", 5);
  stamp_diff_50hz_pub =
    create_publisher<dji_sdk::msg::Int64Stamped>("dji_sdk/stamp_diff/50hz", 50);
  stamp_diff_100hz_pub =
    create_publisher<dji_sdk::msg::Int64Stamped>("dji_sdk/stamp_diff/100hz", 100);
  stamp_diff_400hz_pub =
    create_publisher<dji_sdk::msg::Int64Stamped>("dji_sdk/stamp_diff/400hz", 400);
#ifdef COMPARE_PPS_AND_SOFTSYNC
  hardsync_debug_publisher =
      create_publisher<dji_sdk::msg::HardSyncDebugStamped>("dji_sdk/hardsync_debug", 400);

  packagetimestamp_sub400Hz_debug_publisher =
      create_publisher<dji_sdk::msg::PackageTimestampDebugStamped>("dji_sdk/packagetimestamp_debug/sub400hz", 400);

  packagetimestamp_400Hz_debug_publisher =
      create_publisher<dji_sdk::msg::PackageTimestampDebugStamped>("dji_sdk/packagetimestamp_debug/400hz", 400);

  softsync_400hz_lag_pub =
    create_publisher<dji_sdk::msg::Int64Stamped>("dji_sdk/softsync_400hz_lag_nsec", 400);

  softsync_sub400hz_lag_pub =
    create_publisher<dji_sdk::msg::Int64Stamped>("dji_sdk/softsync_sub_400hz_lag_nsec", 400);
#endif
  control_authority_ack_publisher =
      create_publisher<dji_sdk::msg::UInt32Stamped>("dji_sdk/control_authority_ack", 10);

#ifdef ADVANCED_SENSING
  stereo_240p_front_left_publisher =
    create_publisher<sensor_msgs::msg::Image>("dji_sdk/stereo_240p_front_left_images", 10);

  stereo_240p_front_right_publisher =
    create_publisher<sensor_msgs::msg::Image>("dji_sdk/stereo_240p_front_right_images", 10);

  stereo_240p_down_front_publisher =
    create_publisher<sensor_msgs::msg::Image>("dji_sdk/stereo_240p_down_front_images", 10);

  stereo_240p_down_back_publisher =
    create_publisher<sensor_msgs::msg::Image>("dji_sdk/stereo_240p_down_back_images", 10);

  stereo_240p_front_depth_publisher =
    create_publisher<sensor_msgs::msg::Image>("dji_sdk/stereo_240p_front_depth_images", 10);

  stereo_vga_front_left_publisher =
    create_publisher<sensor_msgs::msg::Image>("dji_sdk/stereo_vga_front_left_images", 10);

  stereo_vga_front_right_publisher =
    create_publisher<sensor_msgs::msg::Image>("dji_sdk/stereo_vga_front_right_images", 10);

  main_camera_stream_publisher =
    create_publisher<sensor_msgs::msg::Image>("dji_sdk/main_camera_images", 10);

  fpv_camera_stream_publisher =
    create_publisher<sensor_msgs::msg::Image>("dji_sdk/fpv_camera_images", 10);
#endif



  if (telemetry_from_fc == USE_BROADCAST)
  {
    ACK::ErrorCode broadcast_set_freq_ack;
    RCLCPP_INFO(get_logger(), "Use legacy data broadcast to get telemetry data!");

    uint8_t defaultFreq[16];

    if(isM100())
    {
      setUpM100DefaultFreq(defaultFreq);
    }
    else
    {
      setUpA3N3DefaultFreq(defaultFreq);
    }

    broadcast_set_freq_ack =
      vehicle->broadcast->setBroadcastFreq(defaultFreq, WAIT_TIMEOUT);
//      vehicle->broadcast->setBroadcastFreqDefaults(WAIT_TIMEOUT);

    if (ACK::getError(broadcast_set_freq_ack))
    {
      ACK::getErrorCodeMessage(broadcast_set_freq_ack, __func__);
      return false;
    }
    // register a callback function whenever a broadcast data is in
    vehicle->broadcast->setUserBroadcastCallback(
      &DJISDKNode::SDKBroadcastCallback, this);
  }
  else if (telemetry_from_fc == USE_SUBSCRIBE)
  {
    RCLCPP_INFO(get_logger(), "Use data subscription to get telemetry data!");
    // Extra topics that is only available from subscription

    // Details can be found in DisplayMode enum in dji_sdk.h
    displaymode_publisher =
      create_publisher<dji_sdk::msg::UInt8Stamped>("dji_sdk/display_mode", 10);

    angularRate_publisher =
      create_publisher<geometry_msgs::msg::Vector3Stamped>("dji_sdk/angular_velocity_fused", 10);

    acceleration_publisher =
      create_publisher<geometry_msgs::msg::Vector3Stamped>("dji_sdk/acceleration_ground_fused", 10);

    baro_height_publisher =
      create_publisher<dji_sdk::msg::BaroHeight>("dji_sdk/barometer_height", 10);

    trigger_publisher = create_publisher<sensor_msgs::msg::TimeReference>("dji_sdk/trigger_time", 10);

    if (!initDataSubscribeFromFC())
    {
      return false;
    }
  }
  vehicle->moc->setFromMSDKCallback(&DJISDKNode::SDKfromMobileDataCallback,
                                    this);
  if (vehicle->payloadDevice)
  {
    vehicle->payloadDevice->setFromPSDKCallback(&DJISDKNode::SDKfromPayloadDataCallback, this);
  }

  if (vehicle->hardSync)
  {
    vehicle->hardSync->subscribeNMEAMsgs(NMEACallback, this);
    vehicle->hardSync->subscribeUTCTime(GPSUTCTimeCallback, this);
    vehicle->hardSync->subscribeFCTimeInUTCRef(FCTimeInUTCCallback, this);
    vehicle->hardSync->subscribePPSSource(PPSSourceCallback, this);
  }
  return true;
}

bool
DJISDKNode::initDataSubscribeFromFC(void)
{
  ACK::ErrorCode ack = vehicle->subscribe->verify(WAIT_TIMEOUT);
  if (ACK::getError(ack))
  {
    return false;
  }

  std::vector<Telemetry::TopicName> topicList100Hz;
  topicList100Hz.push_back(Telemetry::TOPIC_QUATERNION);
  topicList100Hz.push_back(Telemetry::TOPIC_ACCELERATION_GROUND);
  topicList100Hz.push_back(Telemetry::TOPIC_ANGULAR_RATE_FUSIONED);
  topicList100Hz.push_back(Telemetry::TOPIC_ALTITUDE_BAROMETER);

  if (vehicle->subscribe->initPackageFromTopicList(PACKAGE_ID_100HZ, topicList100Hz.size(),
                                                   topicList100Hz.data(), 1, 100))
  {
    ack = vehicle->subscribe->startPackage(PACKAGE_ID_100HZ, WAIT_TIMEOUT);
    if (ACK::getError(ack))
    {
      vehicle->subscribe->removePackage(PACKAGE_ID_100HZ, WAIT_TIMEOUT);
      RCLCPP_ERROR(get_logger(), "Failed to start 100Hz package");
      return false;
    }
    else
    {
      vehicle->subscribe->registerUserPackageUnpackCallback(
              PACKAGE_ID_100HZ, publish100HzData, this);
    }
  }

  std::vector<Telemetry::TopicName> topicList50Hz;
  // 50 Hz package from FC
  topicList50Hz.push_back(Telemetry::TOPIC_GPS_FUSED);
  topicList50Hz.push_back(Telemetry::TOPIC_ALTITUDE_FUSIONED);
  topicList50Hz.push_back(Telemetry::TOPIC_HEIGHT_FUSION);
  topicList50Hz.push_back(Telemetry::TOPIC_STATUS_FLIGHT);
  topicList50Hz.push_back(Telemetry::TOPIC_STATUS_DISPLAYMODE);
  topicList50Hz.push_back(Telemetry::TOPIC_RC);
  topicList50Hz.push_back(Telemetry::TOPIC_VELOCITY);
  topicList50Hz.push_back(Telemetry::TOPIC_GPS_CONTROL_LEVEL);

  if(vehicle->getFwVersion() > versionBase33)
  {
    topicList50Hz.push_back(Telemetry::TOPIC_POSITION_VO);
    topicList50Hz.push_back(Telemetry::TOPIC_RC_WITH_FLAG_DATA);
    topicList50Hz.push_back(Telemetry::TOPIC_FLIGHT_ANOMALY);

    // A3 and N3 has access to more buttons on RC
    std::string hardwareVersion(vehicle->getHwVersion());
    if( (hardwareVersion == std::string(Version::N3)) || hardwareVersion == std::string(Version::A3))
      topicList50Hz.push_back(Telemetry::TOPIC_RC_FULL_RAW_DATA);

    // Advertise rc connection status only if this topic is supported by FW
    rc_connection_status_publisher =
            create_publisher<std_msgs::msg::UInt8>("dji_sdk/rc_connection_status", 10);

    flight_anomaly_publisher =
            create_publisher<dji_sdk::msg::FlightAnomaly>("dji_sdk/flight_anomaly", 10);
  }

  if (vehicle->subscribe->initPackageFromTopicList(PACKAGE_ID_50HZ, topicList50Hz.size(),
                                                   topicList50Hz.data(), true, 50))
  {
    ack = vehicle->subscribe->startPackage(PACKAGE_ID_50HZ, WAIT_TIMEOUT);
    if (ACK::getError(ack))
    {
      vehicle->subscribe->removePackage(PACKAGE_ID_50HZ, WAIT_TIMEOUT);
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to start 50Hz package with ack.data = " << ack.data);
      return false;
    }
    else
    {
      vehicle->subscribe->registerUserPackageUnpackCallback(
              PACKAGE_ID_50HZ, publish50HzData, (UserData) this);
    }
  }

  //! Check if RTK is supported in the FC
  Telemetry::TopicName topicRTKSupport[] =
  {
    Telemetry::TOPIC_RTK_POSITION
  };

  int nTopicRTKSupport    = sizeof(topicRTKSupport)/sizeof(topicRTKSupport[0]);
  if (vehicle->subscribe->initPackageFromTopicList(PACKAGE_ID_5HZ, nTopicRTKSupport,
                                                   topicRTKSupport, 1, 5))
  {
    ack = vehicle->subscribe->startPackage(PACKAGE_ID_5HZ, WAIT_TIMEOUT);
    if (ack.data == ErrorCode::SubscribeACK::SOURCE_DEVICE_OFFLINE)
    {
      rtkSupport = false;
      RCLCPP_INFO(get_logger(), "Flight Controller does not support RTK");
    }
    else
    {
      rtkSupport = true;
      vehicle->subscribe->removePackage(PACKAGE_ID_5HZ, WAIT_TIMEOUT);
    }
  }

  std::vector<Telemetry::TopicName> topicList5hz;
  topicList5hz.push_back(Telemetry::TOPIC_GPS_DATE);
  topicList5hz.push_back(Telemetry::TOPIC_GPS_TIME);
  topicList5hz.push_back(Telemetry::TOPIC_GPS_POSITION);
  topicList5hz.push_back(Telemetry::TOPIC_GPS_VELOCITY);
  topicList5hz.push_back(Telemetry::TOPIC_GPS_DETAILS);
  topicList5hz.push_back(Telemetry::TOPIC_BATTERY_INFO);

  if(rtkSupport)
  {
    topicList5hz.push_back(Telemetry::TOPIC_RTK_POSITION);
    topicList5hz.push_back(Telemetry::TOPIC_RTK_VELOCITY);
    topicList5hz.push_back(Telemetry::TOPIC_RTK_YAW);
    topicList5hz.push_back(Telemetry::TOPIC_RTK_YAW_INFO);
    topicList5hz.push_back(Telemetry::TOPIC_RTK_POSITION_INFO);

    // Advertise rtk data only when rtk is supported
    rtk_position_publisher =
            create_publisher<sensor_msgs::msg::NavSatFix>("dji_sdk/rtk_position", 5);

    rtk_velocity_publisher =
            create_publisher<geometry_msgs::msg::Vector3Stamped>("dji_sdk/rtk_velocity", 5);

    raw_rtk_yaw_publisher =
            create_publisher<std_msgs::msg::Int16>("dji_sdk/raw_rtk_yaw", 5);

    rtk_yaw_publisher =
            create_publisher<dji_sdk::msg::RTKYaw>("dji_sdk/rtk_yaw", 5);

    rtk_position_info_publisher =
            create_publisher<std_msgs::msg::UInt8>("dji_sdk/rtk_info_position", 5);

    rtk_yaw_info_publisher =
            create_publisher<std_msgs::msg::UInt8>("dji_sdk/rtk_info_yaw", 5);

    if(vehicle->getFwVersion() > versionBase33)
    {
      topicList5hz.push_back(Telemetry::TOPIC_RTK_CONNECT_STATUS);

      // Advertise rtk connection only when rtk is supported
      rtk_connection_status_publisher =
              create_publisher<std_msgs::msg::UInt8>("dji_sdk/rtk_connection_status", 5);
    }
  }

  if (vehicle->subscribe->initPackageFromTopicList(PACKAGE_ID_5HZ, topicList5hz.size(),
                                                   topicList5hz.data(), 1, 5))
  {
    ack = vehicle->subscribe->startPackage(PACKAGE_ID_5HZ, WAIT_TIMEOUT);
    if (ACK::getError(ack))
    {
      vehicle->subscribe->removePackage(PACKAGE_ID_5HZ, WAIT_TIMEOUT);
      RCLCPP_ERROR(get_logger(), "Failed to start 5hz package");
      return false;
    }
    else
    {
      vehicle->subscribe->registerUserPackageUnpackCallback(
              PACKAGE_ID_5HZ, publish5HzData, (UserData) this);
    }
  }

  // 400 Hz data from FC
  std::vector<Telemetry::TopicName> topicList400Hz;
  topicList400Hz.push_back(Telemetry::TOPIC_HARD_SYNC);

  if (vehicle->subscribe->initPackageFromTopicList(PACKAGE_ID_400HZ, topicList400Hz.size(),
                                                   topicList400Hz.data(), 1, 400))
  {
    ack = vehicle->subscribe->startPackage(PACKAGE_ID_400HZ, WAIT_TIMEOUT);
    if(ACK::getError(ack))
    {
      vehicle->subscribe->removePackage(PACKAGE_ID_400HZ, WAIT_TIMEOUT);
      RCLCPP_ERROR(get_logger(), "Failed to start 400Hz package");
      return false;
    }
    else
    {
      vehicle->subscribe->registerUserPackageUnpackCallback(PACKAGE_ID_400HZ, publish400HzData, this);
    }
  }
  rclcpp::sleep_for(std::chrono::seconds(1));
  return true;
}

void
DJISDKNode::cleanUpSubscribeFromFC()
{
  vehicle->subscribe->removePackage(0, WAIT_TIMEOUT);
  vehicle->subscribe->removePackage(1, WAIT_TIMEOUT);
  vehicle->subscribe->removePackage(2, WAIT_TIMEOUT);
  vehicle->subscribe->removePackage(3, WAIT_TIMEOUT);
  if (vehicle->hardSync)
  {
    vehicle->hardSync->unsubscribeNMEAMsgs();
    vehicle->hardSync->unsubscribeUTCTime();
    vehicle->hardSync->unsubscribeFCTimeInUTCRef();
    vehicle->hardSync->unsubscribePPSSource();
  }
}

bool DJISDKNode::validateSerialDevice(LinuxSerialDevice* serialDevice)
{
  static const int BUFFER_SIZE = 2048;
  //! Check the serial channel for data
  uint8_t buf[BUFFER_SIZE];
  if (!serialDevice->setSerialPureTimedRead())
  {
    RCLCPP_ERROR(get_logger(), "Failed to set up port for timed read.\n");
    return (false);
  };
  usleep(100000);
  if(serialDevice->serialRead(buf, BUFFER_SIZE))
  {
    RCLCPP_INFO(get_logger(), "Succeeded to read from serial device");
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "Failed to read from serial device. The Onboard SDK is not communicating with your drone.");
    return (false);
  }

  // All the tests passed and the serial device is properly set up
  serialDevice->unsetSerialPureTimedRead();
  return (true);
}

void
DJISDKNode::setUpM100DefaultFreq(uint8_t freq[16])
{
  freq[0]  = DataBroadcast::FREQ_100HZ;
  freq[1]  = DataBroadcast::FREQ_100HZ;
  freq[2]  = DataBroadcast::FREQ_100HZ;
  freq[3]  = DataBroadcast::FREQ_50HZ;
  freq[4]  = DataBroadcast::FREQ_100HZ;
  freq[5]  = DataBroadcast::FREQ_50HZ;
  freq[6]  = DataBroadcast::FREQ_10HZ;
  freq[7]  = DataBroadcast::FREQ_50HZ;
  freq[8]  = DataBroadcast::FREQ_50HZ;
  freq[9]  = DataBroadcast::FREQ_50HZ;
  freq[10] = DataBroadcast::FREQ_10HZ;
  freq[11] = DataBroadcast::FREQ_10HZ;
}

void
DJISDKNode::setUpA3N3DefaultFreq(uint8_t freq[16])
{
  freq[0]  = DataBroadcast::FREQ_100HZ;
  freq[1]  = DataBroadcast::FREQ_100HZ;
  freq[2]  = DataBroadcast::FREQ_100HZ;
  freq[3]  = DataBroadcast::FREQ_50HZ;
  freq[4]  = DataBroadcast::FREQ_100HZ;
  freq[5]  = DataBroadcast::FREQ_50HZ;
  freq[6]  = DataBroadcast::FREQ_50HZ;
  freq[7]  = DataBroadcast::FREQ_50HZ;
  freq[8]  = DataBroadcast::FREQ_10HZ;
  freq[9]  = DataBroadcast::FREQ_50HZ;
  freq[10] = DataBroadcast::FREQ_50HZ;
  freq[11] = DataBroadcast::FREQ_50HZ;
  freq[12] = DataBroadcast::FREQ_10HZ;
  freq[13] = DataBroadcast::FREQ_10HZ;
}

std::string DJISDKNode::controlAuthorityErrorString(const uint32_t error_code)
{
  using ErrorCode = OpenProtocolCMD::ErrorCode::ControlACK::SetControl;
  if (error_code == ErrorCode::RC_MODE_ERROR)
    return "RC_MODE_ERROR";
  else if (error_code == ErrorCode::RELEASE_CONTROL_SUCCESS)
    return "RELEASE_CONTROL_SUCCESS";
  else if (error_code == ErrorCode::OBTAIN_CONTROL_SUCCESS)
    return "OBTAIN_CONTROL_SUCCESS";
  else if (error_code == ErrorCode::OBTAIN_CONTROL_IN_PROGRESS)
    return "OBTAIN_CONTROL_IN_PROGRESS";
  else if (error_code == ErrorCode::RELEASE_CONTROL_IN_PROGRESS)
    return "RELEASE_CONTROL_IN_PROGRESS";
  else if (error_code == ErrorCode::RC_NEED_MODE_F)
    return "RC_NEED_MODE_F";
  else if (error_code == ErrorCode::RC_NEED_MODE_P)
    return "RC_NEED_MODE_P";
  else if (error_code == ErrorCode::IOC_OBTAIN_CONTROL_ERROR)
    return "IOC_OBTAIN_CONTROL_ERROR";
  else
    return "";
}
