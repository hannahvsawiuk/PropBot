#include <propbot_mission/mission_command_interface.hh>

#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <mapviz_plugins/MissionCommandCode.h>
#include <propbot_mission/mission.hh>
#include <propbot_mission/waypoint.hh>
#include <propbot_util/exception.hh>

using namespace propbot;

void MissionCommandInterface::UploadMission(const mapviz_plugins::Mission& mission) {
  std::vector<Waypoint> gps_waypoints;

  for (const auto& waypoint : mission.waypoints) {
    double longitude = waypoint.position.x;
    double latitude = waypoint.position.y;
    gps_waypoints.push_back(Waypoint(std::pair<double, double>(latitude, longitude), utm_zone_));
  }

  // Declare robot mission
  Mission gps_mission(gps_waypoints);

  try {
    mission_handler_ = std::make_unique<MissionHandler>(gps_mission);
  } catch (const propbot::Exception& e) {
    ROS_ERROR("%s", e.what());
  }
}



void MissionCommandInterface::SendMissionCommand(const mapviz_plugins::MissionCommand& mission_command){
  if(mission_handler_) {
    ROS_ERROR("No mission has been uploaded!");
    return;
  }

  using command = mapviz_plugins::MissionCommandCode;

  switch(mission_command.command) {
    case command::MISSION_START: mission_handler_->Start(); break;
    case command::MISSION_PAUSE: mission_handler_->Pause(); break;
    case command::MISSION_RESUME: mission_handler_->Resume(); break;
    case command::MISSION_END: mission_handler_->End();
  }

}