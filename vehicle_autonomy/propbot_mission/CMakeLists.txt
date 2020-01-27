cmake_minimum_required(VERSION 2.8.3)
project(propbot_mission)

set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 17)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  roslib
  roslaunch
)

catkin_package()

roslaunch_add_file_check(launch)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(propbot_mission 
  src/waypoint.cc
  src/mission.cc
  src/mission_handler.cc
)

add_executable(send_mission 
  src/send_mission.cc
)

target_link_libraries(propbot_mission PUBLIC ${catkin_LIBRARIES})
target_link_libraries(send_mission PUBLIC propbot_mission ${catkin_LIBRARIES})

install(DIRECTORY  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
