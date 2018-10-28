cmake_minimum_required(VERSION 2.8.3)
project(dualarmbot_navigation)


find_package(catkin REQUIRED)

catkin_package()

### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  roslaunch_add_file_check(launch)
endif()

### INSTALLS
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY param
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/param
)

install(PROGRAMS
    mockup/move_base_node_mockup
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
