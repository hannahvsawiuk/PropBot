cmake_minimum_required(VERSION 2.8.3)
project(propbot_mission)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  roslib
  roslaunch
  propbot_util
  propbot_common_msgs
)

catkin_package()

roslaunch_add_file_check(launch)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(propbot_mission 
  src/waypoint.cc
  src/mission_handler.cc
  src/mission_command_interface.cc
)

add_executable(execute_mission 
  src/execute_mission.cc
)

target_link_libraries(propbot_mission PUBLIC ${catkin_LIBRARIES})
target_link_libraries(execute_mission PUBLIC propbot_mission ${catkin_LIBRARIES})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
