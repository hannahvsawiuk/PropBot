#ifndef PROPBOT_MISSION_MISSION_
#define PROPBOT_MISSION_MISSION_

#include <memory>

// #include <ros/ros.h>
// #include <ros/package.h>
#include <ros/console.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <propbot_mission/mission.h>
#include <propbot_mission/waypoint.h>

namespace propbot_mission {

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

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

  // Mission commands
  void Start(void);
  void Stop(void) const;

  // Accessors
  Waypoint current_waypoint(void) const;
  unsigned int current_waypoint_number(void) const;

 private:
  // Mission
  Mission mission_;

  // Action client
  std::uniqueptr<MoveBaseClient> action_client_;

  // Index of current waypoint
  unsigned int current_waypoint_index_;

  // Functions
  void SendGoal(void) const;
  void CreateCurrentGoal(void) const;
  void WaypointCallback(void);
};

}  // namespace propbot_mission

#endif  // PROPBOT_MISSION_MISSION_