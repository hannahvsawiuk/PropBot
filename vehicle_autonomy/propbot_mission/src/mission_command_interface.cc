#include <propbot_mission/mission_command_interface.hh>

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <vector>

#include <propbot_common_msgs/MissionCommandCode.h>
#include <propbot_mission/mission.hh>
#include <propbot_mission/waypoint.hh>
#include <propbot_util/exception.hh>

using namespace propbot;

/**
 * Upload mission function
 * 
 * @brief This function instantiates a mission handler and initializes it with the new mission.
 * 
 * @param mission Mission received from mapviz to be uploaded to the robot 
 */
void MissionCommandInterface::UploadMission(
    const propbot_common_msgs::Mission& mission) {
  std::vector<Waypoint> gps_waypoints;

  // Check if a mission handler has been instantiated with a mission
  if (mission_handler_) {
    ROS_ERROR("A mission is already in progress. Cannot upload new mission! Please end current mission.");
    return;
  }

  for (const auto& waypoint : mission.waypoints) {
    double longitude = waypoint.position.x;
    double latitude = waypoint.position.y;
    gps_waypoints.push_back(
        Waypoint(std::pair<double, double>(latitude, longitude), utm_zone_));
  }

  // Declare robot mission
  Mission gps_mission(gps_waypoints);

  ROS_INFO("Uploading mission with %i waypoints...",
           gps_mission.number_waypoints());

  // Instantiate mission handler with new mission
  try {
    mission_handler_ = std::make_unique<MissionHandler>(gps_mission);
  } catch (const propbot::Exception& e) {
    ROS_ERROR("%s", e.what());
    return;
  }

  ROS_INFO("Successfully uploaded mission to Robot.");
}

/**
 * Send Mission Command function
 * 
 * @brief This function uses mapviz mission commands and executes the relevant commands using the mission handler.
 * 
 * @param mission_command Mission command from mapviz including commands such as start, pause, resume, and end mission
 * 
 */
void MissionCommandInterface::SendMissionCommand(
    const propbot_common_msgs::MissionCommand& mission_command) {
  
  // Check if a mission handler has been instantiated with a mission
  if (!mission_handler_) {
    ROS_ERROR("No mission has been uploaded!");
    return;
  }

  using command = propbot_common_msgs::MissionCommandCode;

  // Call mission handler functions based on mission commands
  switch (mission_command.command) {
    case command::MISSION_START:
      mission_handler_->Start();
      break;
    case command::MISSION_PAUSE:
      mission_handler_->Pause();
      break;
    case command::MISSION_RESUME:
      mission_handler_->Resume();
      break;
    case command::MISSION_END:
      mission_handler_->End();
      mission_handler_.reset(nullptr);
  }
}
