cmake_minimum_required(VERSION 3.5)
project(dji_sdk)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# add_definitions(-DCOMPARE_PPS_AND_SOFTSYNC)

## Find ament and other ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# find_package(djiosdk-core QUIET)
find_package(DJIOSDK REQUIRED)
find_package(Threads REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DateTime.msg"
  "msg/DateTimeStamped.msg"
  "msg/FCTimeInUTC.msg"
  "msg/GPSUTC.msg"
  "msg/Gimbal.msg"
  "msg/Waypoint.msg"
  "msg/WaypointList.msg"
  "msg/MobileData.msg"
  "msg/PayloadData.msg"
  "msg/MissionWaypointAction.msg"
  "msg/MissionWaypoint.msg"
  "msg/MissionWaypointTask.msg"
  "msg/MissionHotpointTask.msg"
  "msg/FlightAnomaly.msg"
  "msg/UInt8Stamped.msg"
  "msg/Int64Stamped.msg"
  "msg/UInt32Stamped.msg"
  "msg/VOPosition.msg"
  "msg/RTKYaw.msg"
  "msg/RTKPosition.msg"
  "msg/GPSPosition.msg"
  "msg/GPSHealth.msg"
  "msg/GPSRaw.msg"
  "msg/BaroHeight.msg"
  "msg/HardSyncDebug.msg"
  "msg/HardSyncDebugStamped.msg"
  "msg/PackageTimestampDebug.msg"
  "msg/PackageTimestampDebugStamped.msg"
  "srv/Activation.srv"
  "srv/CameraAction.srv"
  "srv/DroneTaskControl.srv"
  "srv/SDKControlAuthority.srv"
  "srv/SetLocalPosRef.srv"
  "srv/MFIOConfig.srv"
  "srv/MFIOSetValue.srv"
  "srv/DroneArmControl.srv"
  "srv/MissionStatus.srv"
  "srv/MissionWpAction.srv"
  "srv/MissionHpAction.srv"
  "srv/MissionWpUpload.srv"
  "srv/MissionWpSetSpeed.srv"
  "srv/MissionWpGetSpeed.srv"
  "srv/MissionWpGetInfo.srv"
  "srv/MissionHpUpload.srv"
  "srv/MissionHpGetInfo.srv"
  "srv/MissionHpUpdateYawRate.srv"
  "srv/MissionHpUpdateRadius.srv"
  "srv/MissionHpResetYaw.srv"
  "srv/SendMobileData.srv"
  "srv/SendPayloadData.srv"
  "srv/SetHardSync.srv"
  "srv/QueryDroneVersion.srv"
  "srv/Stereo240pSubscription.srv"
  "srv/StereoVGASubscription.srv"
  "srv/StereoDepthSubscription.srv"
  "srv/SetupCameraStream.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs nav_msgs nmea_msgs std_msgs sensor_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${DJIOSDK_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(dji_sdk
#   src/${PROJECT_NAME}/dji_sdk.cpp
# )

add_library(dji_sdk_geometry
  src/dji_sdk_geometry.cpp
)

## Declare a cpp executable
add_executable(dji_sdk_node
  src/main.cpp
  src/modules/dji_sdk_node_control.cpp
  src/modules/dji_sdk_node_services.cpp
  src/modules/dji_sdk_node.cpp
  src/modules/dji_sdk_node_mission_services.cpp
  src/modules/dji_sdk_node_subscriber.cpp
  src/modules/dji_sdk_node_publisher.cpp
  src/modules/dji_sdk_node_mobile_comm.cpp
  src/modules/dji_sdk_node_payload_comm.cpp
  src/modules/dji_sdk_node_time_sync.cpp
)

# Dependencies for the library
ament_target_dependencies(dji_sdk_geometry
  rclcpp
  geometry_msgs
  nav_msgs
  nmea_msgs
  std_msgs
  sensor_msgs
  tf2_ros
)

# Dependencies for the executable
ament_target_dependencies(dji_sdk_node
  rclcpp
  geometry_msgs
  nav_msgs
  nmea_msgs
  std_msgs
  sensor_msgs
  tf2_ros
)

# Add dependency on message generation
rosidl_target_interfaces(dji_sdk_geometry ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(dji_sdk_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

if (NOT TARGET drone_pps)
  add_subdirectory(include/drone_pps)
endif()

## Specify libraries to link a library or executable target against
target_link_libraries(dji_sdk_node
  ${DJIOSDK_LIBRARIES}
  dji_sdk_geometry
  drone_pps
)

#############
## Tests ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(testDJISDKGeometry test/test_dji_sdk_geometry.cpp)
  target_link_libraries(testDJISDKGeometry
    dji_sdk_geometry
    ${DJIOSDK_LIBRARIES}
    ${Boost_LIBRARIES}
  )
  target_include_directories(testDJISDKGeometry PUBLIC
    include
    ${DJIOSDK_INCLUDE_DIRS}
  )
endif()

#############
## Install ##
#############

## Mark executables and libraries for installation
install(TARGETS dji_sdk_node dji_sdk_geometry
  DESTINATION lib/${PROJECT_NAME}
)

## Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS dji_sdk_geometry
  EXPORT dji_sdk_geometryTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_include_directories(include)
ament_export_targets(dji_sdk_geometryTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(rosidl_default_runtime DJIOSDK Threads Boost rclcpp geometry_msgs nav_msgs nmea_msgs std_msgs sensor_msgs tf2_ros)

ament_package()


