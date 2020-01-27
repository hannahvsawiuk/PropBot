#ifndef PROPBOT_MISSION_MISSION_
#define PROPBOT_MISSION_MISSION_

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <propbot_mission/mission.h>

namespace propbot_mission {

/**
 * Mission handler class
 *
 * This class starts a  mission and monitors its progress.
 *
 */
class MissionHandler {
 public:
  /* Default constructor */
  MissionHandler() = default;
  /* Mission handler constructor with a mission as an input */
  MissionHandler(const Mission &mission)
      : mission_(mission), current_waypoint_index_(0){};

  Start();
  Stop();

  Waypoint current_waypoint() const;
  unsigned int current_waypoint_number() const;

 private:
  Mission mission_;

  // Index of current waypoint
  unsigned int current_waypoint_index_;

  CreateCurrentGoal();
  WaypointCallback();
};

}  // namespace propbot_mission

#endif  // PROPBOT_MISSION_MISSION_