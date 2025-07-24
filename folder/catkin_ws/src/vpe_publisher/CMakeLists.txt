cmake_minimum_required(VERSION 3.0.2)
project(vpe_publisher)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_srvs
  tf
  tf2
  tf2_ros
  tf2_geometry_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES vpe_publisher
  CATKIN_DEPENDS roscpp geometry_msgs std_srvs tf tf2 tf2_ros tf2_geometry_msgs
  DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(vpe_publisher src/vpe_publisher.cpp)
target_link_libraries(vpe_publisher ${catkin_LIBRARIES})

install(TARGETS vpe_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
) 