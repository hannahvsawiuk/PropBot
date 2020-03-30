#pragma once

#include <string>

#include <propbot_common_msgs/Mission.h>
#include <propbot_common_msgs/MissionCommand.h>

#include <propbot_mission/mission_handler.hh>

namespace propbot {

/**
 * Mission Command Interface
 *
 * This class interfaces with the mission command centre mapviz plugins and executes missions.
 * 
 */

class MissionCommandInterface {
 public:
  MissionCommandInterface() = delete;
  MissionCommandInterface(std::string utm_zone) : utm_zone_(utm_zone) {}

  void UploadMission(const propbot_common_msgs::Mission& mission);
  void SendMissionCommand(const propbot_common_msgs::MissionCommand& mission_command);

 private:

  // Pointer to a mission handler
  std::unique_ptr<MissionHandler> mission_handler_;

  // UTM Zone config
  std::string utm_zone_;
};

}  // namespace propbots
