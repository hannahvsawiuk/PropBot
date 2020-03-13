#include <string>

#include <ros/package.h>
#include <ros/ros.h>

#include <propbot_mission/mission_command_interface.hh>

// Execute mission node
using namespace propbot;

int main(int argc, char** argv) {
  // Initiation node called execute mission
  ros::init(argc, argv, "execute_mission");
  ros::NodeHandle node_handle;

  // Store mission
  std::string utm_zone;
  ros::param::get("/propbot_mission/utm_zone", utm_zone);

  // Declare mission command interface
  MissionCommandInterface mission_command_interface(utm_zone);

  // Set mission message callback to the upload mission function
  ros::Subscriber mission_sub = node_handle.subscribe(
      "/mapviz/mission", 1000, &MissionCommandInterface::UploadMission,
      &mission_command_interface);

  // Set mission command message callback to the send mission command function 
  ros::Subscriber mission_command_sub = node_handle.subscribe(
      "/mapviz/mission_command", 1000,
      &MissionCommandInterface::SendMissionCommand, &mission_command_interface);

  ros::spin();
  return 0;
}
