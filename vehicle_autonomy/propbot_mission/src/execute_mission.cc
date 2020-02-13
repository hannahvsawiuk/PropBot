#include <string>

#include <ros/package.h>
#include <ros/ros.h>

#include <propbot_mission/mission_command_interface.hh>

using namespace propbot;

int main(int argc, char** argv) {
  // Initiation node called execute mission
  ros::init(argc, argv, "execute_mission");
  ros::NodeHandle node_handle;

  // Store mission
  std::string utm_zone;
  ros::param::get("/propbot_mission/utm_zone", utm_zone);

  MissionCommandInterface mission_command_interface(utm_zone);

  ros::Subscriber mission_sub = node_handle.subscribe(
      "/mapviz/mission", 1000, &MissionCommandInterface::UploadMission,
      &mission_command_interface);

  ros::Subscriber mission_command_sub = node_handle.subscribe(
      "/mapviz/mission_command", 1000,
      &MissionCommandInterface::SendMissionCommand, &mission_command_interface);

  ros::spin();
  return 0;
}