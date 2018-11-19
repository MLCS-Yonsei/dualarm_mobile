cmake_minimum_required(VERSION 2.8.3)
project(dualarm_mobile_slam_navi)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs  
)

### BUild

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

### Install
install(DIRECTORY launch param include
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)