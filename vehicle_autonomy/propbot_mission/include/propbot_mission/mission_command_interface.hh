#pragma once

#include <string>

#include <mapviz_plugins/Mission.h>
#include <mapviz_plugins/MissionCommand.h>

#include <propbot_mission/mission_handler.hh>

namespace propbot {

class MissionCommandInterface {
 public:
  MissionCommandInterface() = delete;
  MissionCommandInterface(std::string utm_zone) : utm_zone_(utm_zone) {}

  void UploadMission(const mapviz_plugins::Mission& mission);
  void SendMissionCommand(const mapviz_plugins::MissionCommand& mission_command);

 private:
  std::unique_ptr<MissionHandler> mission_handler_;
  std::string utm_zone_;
};

}  // namespace propbot