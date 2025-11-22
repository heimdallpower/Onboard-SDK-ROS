/** @file dji_sdk_node_services.cpp
 *  @version 3.7
 *  @date July, 2018
 *
 *  @brief
 *  Implementation of the general services of DJISDKNode
 *
 *  @copyright 2018 DJI. All rights reserved.
 *
 */

#include <dji_sdk/dji_sdk_node.h>

bool
DJISDKNode::droneActivationCallback(const dji_sdk::srv::Activation::Request::SharedPtr request,
                                    dji_sdk::srv::Activation::Response::SharedPtr response)
{
  RCLCPP_DEBUG(get_logger(), "called droneActivationCallback");

  //! @note activation arguments should be specified in launch files
  ACK::ErrorCode ack;
  ack = this->activate(this->app_id, this->enc_key);

  RCLCPP_DEBUG(get_logger(), "ack.info: set=%i id=%i", ack.info.cmd_set, ack.info.cmd_id);
  RCLCPP_DEBUG(get_logger(), "ack.data: %i", ack.data);

  response->cmd_set  = (int)ack.info.cmd_set;
  response->cmd_id   = (int)ack.info.cmd_id;
  response->ack_data = (unsigned int)ack.data;

  if (ACK::getError(ack))
  {
    response->result = false;
    ACK::getErrorCodeMessage(ack, __func__);
  }
  else
  {
    response->result = true;
    RCLCPP_DEBUG(get_logger(), "drone activated");
  }

  return true;
}

bool
DJISDKNode::droneArmCallback(const dji_sdk::srv::DroneArmControl::Request::SharedPtr request,
                             dji_sdk::srv::DroneArmControl::Response::SharedPtr response)
{
  RCLCPP_DEBUG(get_logger(), "called droneArmCallback");

  ACK::ErrorCode ack;

  if (request->arm)
  {
    ack = vehicle->control->armMotors(WAIT_TIMEOUT);
    RCLCPP_DEBUG(get_logger(), "called vehicle->control->armMotors()");
  }
  else
  {
    ack = vehicle->control->disArmMotors(WAIT_TIMEOUT);
    RCLCPP_DEBUG(get_logger(), "called vehicle->control->disArmMotors()");
  }

  RCLCPP_DEBUG(get_logger(), "ack.info: set=%i id=%i", ack.info.cmd_set, ack.info.cmd_id);
  RCLCPP_DEBUG(get_logger(), "ack.data: %i", ack.data);

  response->cmd_set  = (int)ack.info.cmd_set;
  response->cmd_id   = (int)ack.info.cmd_id;
  response->ack_data = (unsigned int)ack.data;

  if (ACK::getError(ack))
  {
    response->result = false;
    ACK::getErrorCodeMessage(ack, __func__);
  }
  else
  {
    response->result = true;
  }

  return true;
}

bool
DJISDKNode::sdkCtrlAuthorityCallback(
  const dji_sdk::srv::SDKControlAuthority::Request::SharedPtr request,
  dji_sdk::srv::SDKControlAuthority::Response::SharedPtr response)
{

  RCLCPP_DEBUG(get_logger(), "called sdkCtrlAuthorityCallback");

  ACK::ErrorCode ack;
  if (request->control_enable)
  {
    ack = vehicle->obtainCtrlAuthority(WAIT_TIMEOUT);
    RCLCPP_DEBUG(get_logger(), "called vehicle->obtainCtrlAuthority");
  }
  else
  {
    ack = vehicle->releaseCtrlAuthority(WAIT_TIMEOUT);
    RCLCPP_DEBUG(get_logger(), "called vehicle->releaseCtrlAuthority");
  }

  RCLCPP_DEBUG(get_logger(), "ack.info: set=%i id=%i", ack.info.cmd_set, ack.info.cmd_id);
  RCLCPP_DEBUG(get_logger(), "ack.data: %i", ack.data);

  response->cmd_set  = (int)ack.info.cmd_set;
  response->cmd_id   = (int)ack.info.cmd_id;
  response->ack_data = (unsigned int)ack.data;

  dji_sdk::msg::UInt32Stamped ack_data_msg;
  ack_data_msg.header.stamp = now();
  ack_data_msg.data = ack.data;
  control_authority_ack_publisher->publish(ack_data_msg);

  if (ACK::getError(ack))
  {
    response->result = false;
    ACK::getErrorCodeMessage(ack, __func__);
    RCLCPP_ERROR(
      get_logger(),
      "[dji_sdk] Control authority error: %s",
      controlAuthorityErrorString(ack.data).c_str()
    );
  }
  else
  {
    response->result = true;
  }

  return true;
}

bool
DJISDKNode::setLocalPosRefCallback(const dji_sdk::srv::SetLocalPosRef::Request::SharedPtr request,
                                     dji_sdk::srv::SetLocalPosRef::Response::SharedPtr response) {
  printf("Currrent GPS health is %d \n",current_gps_health );
  if (current_gps_health > 3)
  {
    local_pos_ref_latitude = current_gps_latitude;
    local_pos_ref_longitude = current_gps_longitude;
    local_pos_ref_altitude = current_gps_altitude;
    RCLCPP_INFO(get_logger(), "Local Position reference has been set.");
    RCLCPP_INFO(get_logger(), "MONITOR GPS HEALTH WHEN USING THIS TOPIC");
    local_pos_ref_set = true;

    // Create message to publish to a topic
    sensor_msgs::msg::NavSatFix localFrameLLA;
    localFrameLLA.latitude = local_pos_ref_latitude;
    localFrameLLA.longitude = local_pos_ref_longitude;
    localFrameLLA.altitude = local_pos_ref_altitude;
    local_frame_ref_publisher->publish(localFrameLLA);

    response->result = true;
  }
  else
  {
    RCLCPP_INFO(get_logger(), "Not enough GPS Satellites. ");
    RCLCPP_INFO(get_logger(), "Cannot set Local Position reference");
    local_pos_ref_set = false;
    response->result = false;
  }

  printf("Currrent RTK health is %d \n",current_rtk_health );
  if (current_rtk_health > RTK_FIX_THRESHOLD)
  {
    local_rtk_pos_ref_latitude = current_rtk_latitude;
    local_rtk_pos_ref_longitude = current_rtk_longitude;
    local_rtk_pos_ref_altitude = current_rtk_altitude;
    RCLCPP_INFO(get_logger(), "Local RTK Position reference has been set.");
    RCLCPP_INFO(get_logger(), "MONITOR RTK HEALTH WHEN USING THIS TOPIC");
    local_rtk_pos_ref_set = true;

    // Create message to publish to a topic
    sensor_msgs::msg::NavSatFix localRTKFrameLLA;
    localRTKFrameLLA.latitude = local_rtk_pos_ref_latitude;
    localRTKFrameLLA.longitude = local_rtk_pos_ref_longitude;
    localRTKFrameLLA.altitude = local_rtk_pos_ref_altitude;
    local_rtk_frame_ref_publisher->publish(localRTKFrameLLA);
    // Don't overwrite the response if it is already false due to bad GPS health
    response->result &= true;
  }
  else
  {
    RCLCPP_INFO(get_logger(), "RTK health is not sufficient. ");
    RCLCPP_INFO(get_logger(), "Cannot set Local RTK Position reference");
    local_rtk_pos_ref_set = false;
  }
  return true;
}

bool
DJISDKNode::droneTaskCallback(const dji_sdk::srv::DroneTaskControl::Request::SharedPtr request,
                              dji_sdk::srv::DroneTaskControl::Response::SharedPtr response)
{

  RCLCPP_DEBUG(get_logger(), "called droneTaskCallback");

  ACK::ErrorCode ack;
  if (request->task == 4)
  {
    // takeoff
    ack = vehicle->control->takeoff(WAIT_TIMEOUT);
    RCLCPP_DEBUG(get_logger(), "called vehicle->control->takeoff()");
  }
  else if (request->task == 6)
  {
    // landing
    ack = vehicle->control->land(WAIT_TIMEOUT);
    RCLCPP_DEBUG(get_logger(), "called vehicle->control->land()");
  }
  else if (request->task == 1)
  {
    // gohome
    ack = vehicle->control->goHome(WAIT_TIMEOUT);
    RCLCPP_DEBUG(get_logger(), "called vehicle->control->goHome()");
  }
  else
  {
    RCLCPP_WARN(get_logger(), "unknown request task in droneTaskCallback");
    response->result = false;
  }

  RCLCPP_DEBUG(get_logger(), "ack.info: set=%i id=%i", ack.info.cmd_set, ack.info.cmd_id);
  RCLCPP_DEBUG(get_logger(), "ack.data: %i", ack.data);

  response->cmd_set  = (int)ack.info.cmd_set;
  response->cmd_id   = (int)ack.info.cmd_id;
  response->ack_data = (unsigned int)ack.data;

  if (ACK::getError(ack))
  {
    ACK::getErrorCodeMessage(ack, __func__);
    response->result = false;
  }
  else
  {
    response->result = true;
  }

  return true;
}

bool
DJISDKNode::cameraActionCallback(const dji_sdk::srv::CameraAction::Request::SharedPtr request,
                                 dji_sdk::srv::CameraAction::Response::SharedPtr response)
{
  RCLCPP_DEBUG(get_logger(), "called cameraActionCallback");

  if (request->camera_action == 0)
  {
    vehicle->camera->shootPhoto();
    response->result = true;
  }
  else if (request->camera_action == 1)
  {
    vehicle->camera->videoStart();
    response->result = true;
  }
  else if (request->camera_action == 2)
  {
    vehicle->camera->videoStop();
    response->result = true;
  }
  else
  {
    RCLCPP_WARN(get_logger(), "unknown request task in cameraActionCallback");
    response->result = false;
  }

  return true;
}

bool
DJISDKNode::MFIOConfigCallback(const dji_sdk::srv::MFIOConfig::Request::SharedPtr request,
                               dji_sdk::srv::MFIOConfig::Response::SharedPtr response)
{
  RCLCPP_DEBUG(get_logger(), "called MFIOConfigCallback");

  vehicle->mfio->config((MFIO::MODE)request->mode,
                        (MFIO::CHANNEL)request->channel,
                        (uint32_t)request->init_on_time_us,
                        (uint16_t)request->pwm_freq, WAIT_TIMEOUT);
  return true;
}

bool
DJISDKNode::MFIOSetValueCallback(const dji_sdk::srv::MFIOSetValue::Request::SharedPtr request,
                                 dji_sdk::srv::MFIOSetValue::Response::SharedPtr response)
{
  RCLCPP_DEBUG(get_logger(), "called MFIOSetValueCallback");

  vehicle->mfio->setValue((MFIO::CHANNEL)request->channel,
                          (uint32_t)request->init_on_time_us, WAIT_TIMEOUT);
  return true;
}

bool
DJISDKNode::setHardsyncCallback(const dji_sdk::srv::SetHardSync::Request::SharedPtr request,
                                dji_sdk::srv::SetHardSync::Response::SharedPtr response)
{
  RCLCPP_DEBUG(get_logger(), "called setHardsyncCallback");
  if (request->frequency == 0)
  {
    RCLCPP_INFO(get_logger(), "Call setSyncFreq with parameters (freq=%d, tag=%d). Will do one "
             "time trigger...",
             request->frequency, request->tag);
    vehicle->hardSync->setSyncFreq(request->frequency, request->tag);
    response->result = true;
    return true;
  }

  // The frequency must be between 0 and 200, and be a divisor of 400
  if (request->frequency > 0 && request->frequency <= 200)
  {
    if (400 % (request->frequency) == 0)
    {
      RCLCPP_INFO(get_logger(), "Call setSyncFreq with parameters (freq=%d, tag=%d).",
               request->frequency, request->tag);
      vehicle->hardSync->setSyncFreq(request->frequency, request->tag);
      response->result = true;
      return true;
    }
  }

  RCLCPP_INFO(get_logger(), "In valid frequency!");
  response->result = false;
  return true;
}

bool DJISDKNode::queryVersionCallback(const dji_sdk::srv::QueryDroneVersion::Request::SharedPtr request,
                                      dji_sdk::srv::QueryDroneVersion::Response::SharedPtr response)
{
  response->version = vehicle->getFwVersion();
  response->hardware = std::string(vehicle->getHwVersion());

  if(response->version == 0)
  {
    RCLCPP_INFO(get_logger(), "Failed to get a valid Firmware version from drone!");
  }

  return true;
}

#ifdef ADVANCED_SENSING
bool
DJISDKNode::stereo240pSubscriptionCallback(const dji_sdk::srv::Stereo240pSubscription::Request::SharedPtr request,
                                           dji_sdk::srv::Stereo240pSubscription::Response::SharedPtr response)
{
  RCLCPP_DEBUG(get_logger(), "called stereo240pSubscriptionCallback");

  if (request->unsubscribe_240p == 1)
  {
    vehicle->advancedSensing->unsubscribeStereoImages();
    response->result = true;
    RCLCPP_INFO(get_logger(), "unsubscribe stereo 240p images");
    return true;
  }

  AdvancedSensing::ImageSelection image_select;
  memset(&image_select, 0, sizeof(AdvancedSensing::ImageSelection));

  if (request->front_right_240p == 1)
    image_select.front_right = 1;

  if (request->front_left_240p == 1)
    image_select.front_left = 1;

  if (request->down_front_240p == 1)
    image_select.down_front = 1;

  if (request->down_back_240p == 1)
    image_select.down_back = 1;

  this->stereo_subscription_success = false;
  vehicle->advancedSensing->subscribeStereoImages(&image_select, &publish240pStereoImage, this);

  rclcpp::sleep_for(std::chrono::seconds(1));

  if (this->stereo_subscription_success == true)
  {
    response->result = true;
  }
  else
  {
    response->result = false;
    RCLCPP_WARN(get_logger(), "Stereo 240p subscription service failed, please check your request content.");
  }

  return true;
}

bool
DJISDKNode::stereoDepthSubscriptionCallback(const dji_sdk::srv::StereoDepthSubscription::Request::SharedPtr request,
                                            dji_sdk::srv::StereoDepthSubscription::Response::SharedPtr response)
{
  RCLCPP_DEBUG(get_logger(), "called stereoDepthSubscriptionCallback");

  if (request->unsubscribe_240p == 1)
  {
    vehicle->advancedSensing->unsubscribeStereoImages();
    response->result = true;
    RCLCPP_INFO(get_logger(), "unsubscribe stereo 240p images");
    return true;
  }

  if (request->front_depth_240p == 1)
  {
    this->stereo_subscription_success = false;
    vehicle->advancedSensing->subscribeFrontStereoDisparity(&publish240pStereoImage, this);
  }
  else
  {
    RCLCPP_WARN(get_logger(), "no depth image is subscribed");
    return true;
  }

  rclcpp::sleep_for(std::chrono::seconds(1));

  if (this->stereo_subscription_success == true)
  {
    response->result = true;
  }
  else
  {
    response->result = false;
    RCLCPP_WARN(get_logger(), "Stereo 240p subscription service failed, please check your request content.");
  }

  return true;
}

bool
DJISDKNode::stereoVGASubscriptionCallback(const dji_sdk::srv::StereoVGASubscription::Request::SharedPtr request,
                                          dji_sdk::srv::StereoVGASubscription::Response::SharedPtr response)
{
  RCLCPP_DEBUG(get_logger(), "called stereoVGASubscriptionCallback");

  if (request->unsubscribe_vga == 1)
  {
    vehicle->advancedSensing->unsubscribeVGAImages();
    response->result = true;
    RCLCPP_INFO(get_logger(), "unsubscribe stereo vga images");
    return true;
  }

  if (request->vga_freq != request->VGA_20_HZ
      && request->vga_freq != request->VGA_10_HZ)
  {
    RCLCPP_ERROR(get_logger(), "VGA subscription frequency is wrong");
    response->result = false;
    return true;
  }

  if (request->front_vga == 1)
  {
    this->stereo_vga_subscription_success = false;
    vehicle->advancedSensing->subscribeFrontStereoVGA(request->vga_freq, &publishVGAStereoImage, this);
    rclcpp::sleep_for(std::chrono::seconds(1));
  }

  if (this->stereo_vga_subscription_success == true)
  {
    response->result = true;
  }
  else
  {
    response->result = false;
    RCLCPP_WARN(get_logger(), "Stereo VGA subscription service failed, please check your request content.");
  }

  return true;
}


bool
DJISDKNode::setupCameraStreamCallback(const dji_sdk::srv::SetupCameraStream::Request::SharedPtr request,
                                      dji_sdk::srv::SetupCameraStream::Response::SharedPtr response)
{
  RCLCPP_DEBUG(get_logger(), "called cameraStreamCallback");
  bool result = false;

  if(request->camera_type == request->FPV_CAM)
  {
    if(request->start == 1)
    {
      result = vehicle->advancedSensing->startFPVCameraStream(&publishFPVCameraImage, this);
    }
    else
    {
      vehicle->advancedSensing->stopFPVCameraStream();
      result = true;
    }
  }
  else if(request->camera_type == request->MAIN_CAM)
  {
    if(request->start == 1)
    {
      result = vehicle->advancedSensing->startMainCameraStream(&publishMainCameraImage, this);
    }
    else
    {
      vehicle->advancedSensing->stopMainCameraStream();
      result = true;
    }
  }

  response->result = result;
  return true;
}

#endif // ADVANCED_SENSING
