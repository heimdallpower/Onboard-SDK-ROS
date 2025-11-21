/** @file main.cpp
 *  @version 3.7
 *  @date July, 2018
 *
 *  @brief
 *  DJISDKNode
 *
 *  @copyright 2018 DJI. All rights reserved.
 *
 */

#include <dji_sdk/dji_sdk_node.h>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node{std::make_shared<DJISDKNODE>("dji_sdk")};
  rclcpp::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
  rclcpp::waitForShutdown();
  return 0;
}
