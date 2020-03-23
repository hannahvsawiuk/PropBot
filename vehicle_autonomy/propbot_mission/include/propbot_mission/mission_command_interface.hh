#pragma once

#include <string>

#include <mapviz_plugins/Mission.h>
#include <mapviz_plugins/MissionCommand.h>

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

  void UploadMission(const mapviz_plugins::Mission& mission);
  void SendMissionCommand(const mapviz_plugins::MissionCommand& mission_command);

 private:

  // Pointer to a mission handler
  std::unique_ptr<MissionHandler> mission_handler_;

  // UTM Zone config
  std::string utm_zone_;
};

}  // namespace propbots
