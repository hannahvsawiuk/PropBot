cmake_minimum_required(VERSION 2.8.3)
project(propbot_state_estimation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslaunch
)

catkin_package()

roslaunch_add_file_check(launch)

install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
