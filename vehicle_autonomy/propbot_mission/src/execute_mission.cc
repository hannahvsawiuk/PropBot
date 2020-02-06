#include <string>

#include <ros/package.h>
#include <ros/ros.h>

#include <propbot_mission/mission.hh>
#include <propbot_mission/mission_handler.hh>
#include <propbot_util/exception.hh>

// Send mission node
using namespace propbot;

int main(int argc, char** argv) {
  // Initiation node called send mission
  ros::init(argc, argv, "execute_mission");
  ros::NodeHandle node_handle;

  // Store mission
  std::string mission_file, utm_zone;
  ros::param::get("/propbot_mission/mission_file", mission_file);
  ros::param::get("/propbot_mission/utm_zone", utm_zone);

  mission_file = ros::package::getPath("propbot_mission") + mission_file;
  ROS_INFO("Reading mission from file '%s'...", mission_file.c_str());

  // Instantiate a mission handler
  try {
    Mission mission(mission_file, utm_zone);
    MissionHandler mission_handler(mission);
    // Start mission
    mission_handler.Start();

    while (!mission_handler.Finished()) {
      // Wait for mission to finish
    }

    if (mission_handler.Failed()) {
      ROS_ERROR("Mission failed!");
    }

    ROS_INFO("Mission ended. Killing node...");
  } catch (const Exception& e) {
    ROS_ERROR("%s", e.what());
  }

  ros::shutdown();
  return 0;
}