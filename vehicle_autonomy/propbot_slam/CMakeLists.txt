cmake_minimum_required(VERSION 2.8.3)
project(propbot_slam)

find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES propbot_slam
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
