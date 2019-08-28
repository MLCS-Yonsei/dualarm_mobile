cmake_minimum_required(VERSION 2.8.3)
project(dualarm_mobile_bringup)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  nodelet
  tf
)

  generate_messages(
    DEPENDENCIES
     std_msgs
  )


catkin_package(
  LIBRARIES 
    dualarm_mobile_bringup
  CATKIN_DEPENDS 
    geometry_msgs 
    nav_msgs 
    roscpp 
    std_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES 
    vehicle_v2_nodelet
  CATKIN_DEPENDS 
    rospy 
    roscpp
    geometry_msgs 
    nav_msgs 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(bringup_publisher src/bringup_publisher.cpp)
target_link_libraries(bringup_publisher ${catkin_LIBRARIES})

add_executable(odom_publisher src/odom_publisher.cpp)
target_link_libraries(odom_publisher ${catkin_LIBRARIES})
