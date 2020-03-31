cmake_minimum_required(VERSION 3.0.2)
project(propbot_common_msgs)

# set(DEPENDENCIES
#   std_msgs 
#   geometry_msgs
# )


find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs )

add_message_files(
  FILES
  Mission.msg
  MissionCommand.msg
  MissionCommandCode.msg
)

generate_messages(
  DEPENDENCIES geometry_msgs std_msgs ) 

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)
