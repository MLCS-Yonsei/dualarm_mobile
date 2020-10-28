cmake_minimum_required(VERSION 2.8.3)
project(dualarm_mobile_localization)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs  
)

### BUild

include_directories(
  ${catkin_INCLUDE_DIRS}
)

### Install
install(DIRECTORY launch params
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
