/** @file dji_sdk_node_mission_services.cpp
 *  @version 3.7
 *  @date July, 2018
 *
 *  @brief
 *  Implementation of the mission functions of DJISDKNode
 *
 *  @copyright 2018 DJI. All rights reserved.
 *
 */

#include <dji_sdk/dji_sdk_node.h>

bool
DJISDKNode::missionStatusCallback(const dji_sdk::srv::MissionStatus::Request::SharedPtr request,
                                  dji_sdk::srv::MissionStatus::Response::SharedPtr response)
{
  RCLCPP_DEBUG(get_logger(), "called missionStatusCallback");

  response.waypoint_mission_count = vehicle->missionManager->wpMissionVector.size();
  response.hotpoint_mission_count = vehicle->missionManager->hpMissionVector.size();
  return true;
}

bool
DJISDKNode::missionWpUploadCallback(
  const dji_sdk::srv::MissionWpUpload::Request::SharedPtr request,
  dji_sdk::srv::MissionWpUpload::Response::SharedPtr response)
{
  RCLCPP_DEBUG(get_logger(), "called missionWpUpload");

  //! initialize waypoint mission related info
  ACK::ErrorCode                  initAck;
  DJI::OSDK::WayPointInitSettings wpInitData;
  wpInitData.indexNumber  = (unsigned char)request.waypoint_task.mission_waypoint.size();
  wpInitData.maxVelocity  = (float)request.waypoint_task.velocity_range;
  wpInitData.idleVelocity = (float)request.waypoint_task.idle_velocity;
  wpInitData.finishAction = (unsigned char)request.waypoint_task.action_on_finish;
  wpInitData.executiveTimes = (unsigned char)request.waypoint_task.mission_exec_times;
  wpInitData.yawMode        = (unsigned char)request.waypoint_task.yaw_mode;
  wpInitData.traceMode      = (unsigned char)request.waypoint_task.trace_mode;
  wpInitData.RCLostAction   = (unsigned char)request.waypoint_task.action_on_rc_lost;
  wpInitData.gimbalPitch    = (unsigned char)request.waypoint_task.gimbal_pitch_mode;
  wpInitData.latitude = 0.0;
  wpInitData.longitude = 0.0;
  wpInitData.altitude = 0.0;
  for (int i = 0; i < 16; i++){  
    wpInitData.reserved[i] = 0;
  } 

  initAck = vehicle->missionManager->init(DJI_MISSION_TYPE::WAYPOINT,
                                          WAIT_TIMEOUT, &wpInitData);

  RCLCPP_DEBUG(get_logger(), "ack.info: set=%i id=%i", initAck.info.cmd_set,
            initAck.info.cmd_id);
  RCLCPP_DEBUG(get_logger(), "ack.data: %i", initAck.data);

  response.cmd_set  = (int)initAck.info.cmd_set;
  response.cmd_id   = (int)initAck.info.cmd_id;
  response.ack_data = (unsigned int)initAck.data;

  if (ACK::getError(initAck))
  {
    ACK::getErrorCodeMessage(initAck, __func__);
    response.result = false;
  }

  RCLCPP_INFO(get_logger(), "initialized waypoint mission");
  sleep(1);

  //! initialize waypoint mission related info
  ACK::WayPointIndex          uploadAck;
  DJI::OSDK::WayPointSettings wpData;
  int                         i = 0;
  for (auto waypoint : request.waypoint_task.mission_waypoint)
  {
    wpData.latitude        = waypoint.latitude  * C_PI / 180;
    wpData.longitude       = waypoint.longitude * C_PI / 180;
    wpData.altitude        = waypoint.altitude;
    wpData.damping         = waypoint.damping_distance;
    wpData.yaw             = waypoint.target_yaw;
    wpData.gimbalPitch     = waypoint.target_gimbal_pitch;
    wpData.turnMode        = waypoint.turn_mode;
    wpData.hasAction       = waypoint.has_action;
    wpData.actionTimeLimit = waypoint.action_time_limit;
    wpData.actionNumber    = 15;
    wpData.actionRepeat    = waypoint.waypoint_action.action_repeat;
    wpData.index           = i;
    std::copy(waypoint.waypoint_action.command_list.begin(),
              waypoint.waypoint_action.command_list.end(), wpData.commandList);
    std::copy(waypoint.waypoint_action.command_parameter.begin(),
              waypoint.waypoint_action.command_parameter.end(),
              wpData.commandParameter);

    uploadAck = vehicle->missionManager->wpMission->uploadIndexData(
      &wpData, WAIT_TIMEOUT);

    RCLCPP_DEBUG(get_logger(), "uploaded waypoint lat: %f lon: %f alt: %f", waypoint.latitude,
              waypoint.longitude, waypoint.altitude);

    response.cmd_set  = (int)uploadAck.ack.info.cmd_set;
    response.cmd_id   = (int)uploadAck.ack.info.cmd_id;
    response.ack_data = (unsigned int)uploadAck.ack.data;

    if (ACK::getError(uploadAck.ack))
    {
      ACK::getErrorCodeMessage(uploadAck.ack, __func__);
      response.result = false;
    }
    else
    {
      response.result = true;
    }

    RCLCPP_INFO(get_logger(), "uploaded the %dth waypoint\n", (wpData.index + 1));
    i += 1;
    sleep(1);
  }

  RCLCPP_INFO(get_logger(), "waypoint mission initialized and uploaded");
  return true;
}

bool
DJISDKNode::missionWpActionCallback(
  const dji_sdk::srv::MissionWpAction::Request::SharedPtr request,
  dji_sdk::srv::MissionWpAction::Response::SharedPtr response)
{
  RCLCPP_DEBUG(get_logger(), "called missionWpActionCallback");

  if (vehicle->missionManager->wpMissionVector.size() == 0)
  {
    RCLCPP_ERROR(get_logger(), "no waypoint mission uploaded");
    response->result = false;
  }

  ACK::ErrorCode ack;
  switch (request->action)
  {
    case DJI::OSDK::MISSION_ACTION::START:
      ack = vehicle->missionManager->wpMission->start(WAIT_TIMEOUT);
      RCLCPP_DEBUG(get_logger(), "start waypoint mission");
      break;
    case DJI::OSDK::MISSION_ACTION::STOP:
      ack = vehicle->missionManager->wpMission->stop(WAIT_TIMEOUT);
      RCLCPP_DEBUG(get_logger(), "stop waypoint mission");
      break;
    case DJI::OSDK::MISSION_ACTION::PAUSE:
      ack = vehicle->missionManager->wpMission->pause(WAIT_TIMEOUT);
      RCLCPP_DEBUG(get_logger(), "pause waypoint mission");
      break;
    case DJI::OSDK::MISSION_ACTION::RESUME:
      ack = vehicle->missionManager->wpMission->resume(WAIT_TIMEOUT);
      RCLCPP_DEBUG(get_logger(), "resume waypoint mission");
      break;
    default:
      RCLCPP_WARN(get_logger(), "unknown action specified in MissionWpAction service");
      break;
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
DJISDKNode::missionWpGetSpeedCallback(
  const dji_sdk::srv::MissionWpGetSpeed::Request::SharedPtr request,
  dji_sdk::srv::MissionWpGetSpeed::Response::SharedPtr response)
{
  RCLCPP_DEBUG(get_logger(), "called wpGetSpeedCallback");

  if (vehicle->missionManager->wpMissionVector.size() > 0)
  {
    //! @todo bug here
    //    response.speed =
    //      (vehicle->missionManager->wpMission->readIdleVelocity(WAIT_TIMEOUT))
    //        .idleVelocity;
    //    vehicle->missionManager->wpMission->readIdleVelocity();
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "no waypoint mission initiated ");
  }
  // @todo some bug in FC side, need to follow up
  std::cout << "response.speed " << response->speed << std::endl;

  return true;
}

bool
DJISDKNode::missionWpSetSpeedCallback(
  const dji_sdk::srv::MissionWpSetSpeed::Request::SharedPtr request,
  dji_sdk::srv::MissionWpSetSpeed::Response::SharedPtr response)
{
  RCLCPP_DEBUG(get_logger(), "called wpSetSpeedCallback");

  ACK::WayPointVelocity velAck;

  if (vehicle->missionManager->wpMissionVector.size() > 0)
  {
    velAck = (vehicle->missionManager->wpMission->updateIdleVelocity(
      request->speed, WAIT_TIMEOUT));
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "no waypoint mission initiated ");
    response->result = false;
  }

  if (ACK::getError(velAck.ack))
  {
    RCLCPP_DEBUG(get_logger(), "wpSetSpeedCallback ack value: %d", (uint32_t)velAck.ack.data);
    response->result = false;
  }
  else
  {
    response->result = true;
  }

  return true;
}

bool
DJISDKNode::missionWpGetInfoCallback(
  const dji_sdk::srv::MissionWpGetInfo::Request::SharedPtr request,
  dji_sdk::srv::MissionWpGetInfo::Response::SharedPtr response)
{
  RCLCPP_DEBUG(get_logger(), "called missionWpGetInfoCallback");

  DJI::OSDK::WayPointInitSettings info;
  if (vehicle->missionManager->wpMissionVector.size() > 0)
  {
    info = vehicle->missionManager->wpMission->getWaypointSettings(10).data;
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "no waypoint mission initiated ");
    return false;
  }

  response->waypoint_task.mission_waypoint.resize(info.indexNumber);
  response->waypoint_task.velocity_range     = info.maxVelocity;
  response->waypoint_task.idle_velocity      = info.idleVelocity;
  response->waypoint_task.action_on_finish   = info.finishAction;
  response->waypoint_task.mission_exec_times = info.executiveTimes;
  response->waypoint_task.yaw_mode           = info.yawMode;
  response->waypoint_task.trace_mode         = info.traceMode;
  response->waypoint_task.action_on_rc_lost  = info.RCLostAction;
  response->waypoint_task.gimbal_pitch_mode  = info.gimbalPitch;

  for (int i=0; i< info.indexNumber; i++)
  {
    DJI::OSDK::WayPointSettings wpData; 
    wpData = vehicle->missionManager->wpMission->getIndex(i, 10).data;
    response->waypoint_task.mission_waypoint[i].latitude            = wpData.latitude  * 180.0 / C_PI;
    response->waypoint_task.mission_waypoint[i].longitude           = wpData.longitude * 180.0 / C_PI;
    response->waypoint_task.mission_waypoint[i].altitude            = wpData.altitude;
    response->waypoint_task.mission_waypoint[i].damping_distance    = wpData.damping;
    response->waypoint_task.mission_waypoint[i].target_yaw          = wpData.yaw;
    response->waypoint_task.mission_waypoint[i].target_gimbal_pitch = wpData.gimbalPitch;
    response->waypoint_task.mission_waypoint[i].turn_mode           = wpData.turnMode;
    response->waypoint_task.mission_waypoint[i].has_action          = wpData.hasAction;
    response->waypoint_task.mission_waypoint[i].action_time_limit   = wpData.actionTimeLimit;
    response->waypoint_task.mission_waypoint[i].waypoint_action.action_repeat = wpData.actionNumber + (wpData.actionRepeat << 4);

    std::copy(std::begin(wpData.commandList),
              std::end(wpData.commandList), 
              response->waypoint_task.mission_waypoint[i].waypoint_action.command_list.begin());

    std::copy(std::begin(wpData.commandParameter),
              std::end(wpData.commandParameter), 
              response->waypoint_task.mission_waypoint[i].waypoint_action.command_parameter.begin());

  }
  return true;
}

bool
DJISDKNode::missionHpUploadCallback(
  const dji_sdk::srv::MissionHpUpload::Request::SharedPtr request,
  dji_sdk::srv::MissionHpUpload::Response::SharedPtr response)
{
  RCLCPP_DEBUG(get_logger(), "called missionHpUploadCallback");

  DJI::OSDK::HotPointSettings* hpInitData = new DJI::OSDK::HotPointSettings();
  hpInitData->latitude   = request->hotpoint_task.latitude * C_PI / 180;
  hpInitData->longitude  = request->hotpoint_task.longitude * C_PI / 180;
  hpInitData->height     = request->hotpoint_task.altitude;
  hpInitData->radius     = request->hotpoint_task.radius;
  hpInitData->yawRate    = request->hotpoint_task.angular_speed;
  hpInitData->clockwise  = request->hotpoint_task.is_clockwise;
  hpInitData->startPoint = request->hotpoint_task.start_point;
  hpInitData->yawMode    = request->hotpoint_task.yaw_mode;

  vehicle->missionManager->init(DJI_MISSION_TYPE::HOTPOINT, WAIT_TIMEOUT,
                                (void*)hpInitData);

  response->result = true;
  return true;
}

bool
DJISDKNode::missionHpActionCallback(
  const dji_sdk::srv::MissionHpAction::Request::SharedPtr request,
  dji_sdk::srv::MissionHpAction::Response::SharedPtr response)
{
  RCLCPP_DEBUG(get_logger(), "called missionHpActionCallback");

  if (vehicle->missionManager->hpMissionVector.size() == 0)
  {
    RCLCPP_ERROR(get_logger(), "no hotpoint mission uploaded");
    response->result = false;
  }

  ACK::ErrorCode ack;
  switch (request->action)
  {
    case DJI::OSDK::MISSION_ACTION::START:
      ack = vehicle->missionManager->hpMission->start(WAIT_TIMEOUT);
      RCLCPP_DEBUG(get_logger(), "start hotpoint mission");
      break;
    case DJI::OSDK::MISSION_ACTION::STOP:
      ack = vehicle->missionManager->hpMission->stop(WAIT_TIMEOUT);
      RCLCPP_DEBUG(get_logger(), "stop hotpoint mission");
      break;
    case DJI::OSDK::MISSION_ACTION::PAUSE:
      ack = vehicle->missionManager->hpMission->pause(WAIT_TIMEOUT);
      RCLCPP_DEBUG(get_logger(), "pause hotpoint mission");
      break;
    case DJI::OSDK::MISSION_ACTION::RESUME:
      ack = vehicle->missionManager->hpMission->resume(WAIT_TIMEOUT);
      RCLCPP_DEBUG(get_logger(), "resume hotpoint mission");
      break;
    default:
      RCLCPP_WARN(get_logger(), "unknown action specified in MissionHpAction service");
      break;
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
DJISDKNode::missionHpGetInfoCallback(
  const dji_sdk::srv::MissionHpGetInfo::Request::SharedPtr request,
  dji_sdk::srv::MissionHpGetInfo::Response::SharedPtr response)
{
  RCLCPP_DEBUG(get_logger(), "called missionHpGetInfoCallback");

  DJI::OSDK::HotPointSettings info;
  if (vehicle->missionManager->hpMissionVector.size() > 0)
  {
    info = vehicle->missionManager->hpMission->getData();
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "no hotpoint mission initiated ");
  }

  response->hotpoint_task.latitude      = info.latitude  * 180.0 / C_PI;
  response->hotpoint_task.longitude     = info.longitude * 180.0 / C_PI;
  response->hotpoint_task.altitude      = info.height;
  response->hotpoint_task.radius        = info.radius;
  response->hotpoint_task.angular_speed = info.yawRate;
  response->hotpoint_task.is_clockwise  = info.clockwise;
  response->hotpoint_task.start_point   = info.startPoint;
  response->hotpoint_task.yaw_mode      = info.yawMode;

  return true;
}

bool
DJISDKNode::missionHpUpdateYawRateCallback(
  const dji_sdk::srv::MissionHpUpdateYawRate::Request::SharedPtr request,
  dji_sdk::srv::MissionHpUpdateYawRate::Response::SharedPtr response)
{
  RCLCPP_DEBUG(get_logger(), "called missionHpUpdateYawRateCallback");

  DJI::OSDK::HotpointMission::YawRate yawRate;
  yawRate.yawRate   = request->yaw_rate;
  yawRate.clockwise = request->direction;

  ACK::ErrorCode ack;
  if (vehicle->missionManager->hpMissionVector.size() > 0)
  {
    ack =
      vehicle->missionManager->hpMission->updateYawRate(yawRate, WAIT_TIMEOUT);
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "no hotpoint mission initiated ");
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
DJISDKNode::missionHpResetYawCallback(
  const dji_sdk::srv::MissionHpResetYaw::Request::SharedPtr request,
  dji_sdk::srv::MissionHpResetYaw::Response::SharedPtr response)
{
  RCLCPP_DEBUG(get_logger(), "called missionHpResetYawCallback");

  ACK::ErrorCode ack;
  if (vehicle->missionManager->hpMissionVector.size() > 0)
  {
    ack = vehicle->missionManager->hpMission->resetYaw(WAIT_TIMEOUT);
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "no hotpoint mission initiated ");
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
DJISDKNode::missionHpUpdateRadiusCallback(
  const dji_sdk::srv::MissionHpUpdateRadius::Request::SharedPtr request,
  dji_sdk::srv::MissionHpUpdateRadius::Response::SharedPtr response)
{
  RCLCPP_DEBUG(get_logger(), "called missionHpUpdateRadiusCallback");

  ACK::ErrorCode ack;
  if (vehicle->missionManager->hpMissionVector.size() > 0)
  {
    ack = vehicle->missionManager->hpMission->updateRadius(request->radius,
                                                           WAIT_TIMEOUT);
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "no hotpoint mission initiated ");
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
