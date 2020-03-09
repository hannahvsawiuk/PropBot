cmake_minimum_required(VERSION 2.8.3)
project(propbot_slam)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  roslib
  roslaunch
  geometry_msgs
)


catkin_package()

roslaunch_add_file_check(launch)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(publish_slam_pose 
  src/publish_slam_pose.cc
)

target_link_libraries(publish_slam_pose PUBLIC ${catkin_LIBRARIES})


install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
