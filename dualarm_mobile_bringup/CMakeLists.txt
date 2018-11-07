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

add_executable(vehicle_controller src/vehicle_controller.cpp)
target_link_libraries(vehicle_controller ${catkin_LIBRARIES})
#add_dependencies(vehicle_controller vehicle_control_generate_messages_cpp)
#add_dependencies(vehicle_controller ${catkin_EXPORTED_TARGETS})
# add_dependencies(vehicle_controller ${catkin_EXPORTED_TARGETS})

#add_executable(odom_publisher src/odometry.cpp)
#target_link_libraries(odom_publisher ${catkin_LIBRARIES})

#add_executable(scan_publisher src/scan_publisher.cpp)
#target_link_libraries(scan_publisher ${catkin_LIBRARIES})

#add_executable(tf_broadcaster src/tf_broadcaster.cpp)
#target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})

#add_executable(time_publisher src/time_publisher.cpp)
#target_link_libraries(time_publisher ${catkin_LIBRARIES})

add_executable(bringup_publisher src/bringup_publisher.cpp)
target_link_libraries(bringup_publisher ${catkin_LIBRARIES})

