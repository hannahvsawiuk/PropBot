cmake_minimum_required(VERSION 2.8.3)
project(propbot_object_detection)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roslaunch
  roscpp
  roslib
  std_msgs
  people_msgs
  geometry_msgs
  sensor_msgs
  darknet_ros_msgs
  visualization_msgs
  tf
  image_geometry
  image_transport
  cv_bridge
  propbot_util
)

catkin_package()

roslaunch_add_file_check(launch)

add_executable(publish_people
  src/publish_people.cc
  src/bounding_box_transformer.cc
)

target_include_directories(publish_people PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
)


target_link_libraries(publish_people PUBLIC
  ${catkin_LIBRARIES}
)

install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
