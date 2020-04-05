cmake_minimum_required(VERSION 2.8.3)
project(propbot_object_detection)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  roslib
  roslaunch
  geometry_msgs
  darknet_ros_msgs
  people_msgs
)

catkin_package()

roslaunch_add_file_check(launch)

add_executable(publish_people
  src/publish_people.cc
)

target_include_directories(publish_people PUBLIC
  ${catkin_INCLUDE_DIRS}
)


target_link_libraries(publish_people PUBLIC
  ${catkin_LIBRARIES}
)

install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
