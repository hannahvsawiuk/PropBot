#include <string>

#include <ros/package.h>
#include <ros/ros.h>

#include <propbot_mission/mission.h>
#include <propbot_mission/mission_handler.h>

// Send mission node
using namespace propbot_mission;

int main(int argc, char** argv) {
  // Initiation node called send mission
  ros::init(argc, argv, "execute_mission");
  ros::NodeHandle node_handle;

  // Store mission
  std::string mission_file;
  ros::param::get("/propbot_mission/mission_file", mission_file);
  mission_file = ros::package::getPath("propbot_mission") + mission_file;
  ROS_INFO("Reading mission from file '%s'...", mission_file.c_str());
  Mission mission(mission_file);

  // Instantiate a mission handler
  MissionHandler mission_handler(mission);

  // Start mission
  mission_handler.Start();

  while (!mission_handler.IsFinished()) {
  }

  ROS_INFO("Mission ended. Killing node...");
  ros::shutdown();
  return 0;
}