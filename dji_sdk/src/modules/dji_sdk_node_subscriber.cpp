/** @file dji_sdk_node_subscriber.cpp
 *  @version 3.7
 *  @date July, 2018
 *
 *  @brief
 *  Implementation of the subscribers of DJISDKNode
 *
 *  @copyright 2018 DJI. All rights reserved.
 *
 */

#include <dji_sdk/dji_sdk_node.h>

void
DJISDKNode::gimbalAngleCtrlCallback(const dji_sdk::msg::Gimbal::SharedPtr msg)
{
  RCLCPP_DEBUG(get_logger(), "called gimbalAngleCtrlCallback");

  DJI::OSDK::Gimbal::AngleData angle_data;
  //! OSDK takes 0.1 sec as unit
  angle_data.duration = msg->ts*10;
  angle_data.mode     = msg->mode;
  //! OSDK takes 0.1 deg as unit
  angle_data.roll     = rad2deg(msg->roll)*10;
  angle_data.pitch    = rad2deg(msg->pitch)*10;
  angle_data.yaw      = rad2deg(msg->yaw)*10;
  vehicle->gimbal->setAngle(&angle_data);
}

void
DJISDKNode::gimbalSpeedCtrlCallback(
  const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
  RCLCPP_DEBUG(get_logger(), "called gimbalSpeedCtrlCallback");

  DJI::OSDK::Gimbal::SpeedData speed_data;
  //! OSDK takes 0.1 deg as unit
  speed_data.gimbal_control_authority = 1;
  speed_data.roll  = rad2deg(msg->vector.x)*10;
  speed_data.pitch = rad2deg(msg->vector.y)*10;
  speed_data.yaw   = rad2deg(msg->vector.z)*10;
  vehicle->gimbal->setSpeed(&speed_data);
}
