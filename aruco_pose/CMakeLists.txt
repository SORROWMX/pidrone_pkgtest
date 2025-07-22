cmake_minimum_required(VERSION 3.0.2)
project(aruco_pose)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_generation
  dynamic_reconfigure
)

## Find OpenCV
find_package(OpenCV REQUIRED)

## Generate messages
add_message_files(
  FILES
  Point2D.msg
  Marker.msg
  MarkerArray.msg
)

## Generate services
add_service_files(
  FILES
  SetMarkers.srv
)

## Generate dynamic reconfigure parameters
generate_dynamic_reconfigure_options(
  cfg/Detector.cfg
  cfg/Map.cfg
)

## Generate added messages and services with any dependencies
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs visualization_msgs tf2 tf2_ros tf2_geometry_msgs message_runtime dynamic_reconfigure
)

## Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Install Python scripts
catkin_install_python(PROGRAMS
  src/aruco_detect.py
  src/aruco_map.py
  src/aruco_draw.py
  src/aruco_utils.py
  src/__init__.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Create Python package structure
install(FILES
  src/__init__.py
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/aruco_pose/src
)

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
) 