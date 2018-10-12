cmake_minimum_required(VERSION 2.8.3)
project(dualarmbot_bringup)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  std_msgs
  nodelet
  tf
)

add_message_files(
  FILES
  commendMsg.msg
  motorsMsg.msg
  jointstatesMsg.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  LIBRARIES 
  bringup_dual
  CATKIN_DEPENDS 
  geometry_msgs 
  nav_msgs 
  roscpp 
  std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#add_executable(cmd_publish src/cmd_publish.cpp)
#target_link_libraries(cmd_publish ${catkin_LIBRARIES})
#add_dependencies(cmd_publish vehicle_control_generate_messages_cpp)

add_executable(vehicle_controller src/vehicle_controller.cpp)
target_link_libraries(vehicle_controller ${catkin_LIBRARIES})
add_dependencies(vehicle_controller vehicle_control_generate_messages_cpp)

