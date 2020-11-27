cmake_minimum_required(VERSION 2.8.3)
project(dualarm_mobile_bringup)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  nodelet
  tf
  ethercat_test
  teb_local_planner
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dualarm_mobile_bringup
  CATKIN_DEPENDS 
    roscpp
    geometry_msgs 
    nav_msgs
    std_msgs
    tf
    ethercat_test
    teb_local_planner
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(bringup_publisher src/bringup_publisher.cpp)
target_link_libraries(bringup_publisher ${catkin_LIBRARIES})

add_executable(odom_publisher src/odom_publisher.cpp)
target_link_libraries(odom_publisher ${catkin_LIBRARIES})

add_executable(cmd_vel_publisher src/cmd_vel_publisher.cpp)
target_link_libraries(cmd_vel_publisher ${catkin_LIBRARIES})
