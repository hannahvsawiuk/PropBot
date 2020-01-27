#pragma once

#include <functional>
#include <memory>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>

#include <propbot_mission/mission.h>
#include <propbot_mission/waypoint.h>

namespace propbot_mission {

using MoveBaseClient =
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

using ResultConstPtr = boost::shared_ptr<
    const move_base_msgs::MoveBaseAction::_action_result_type::_result_type>;

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
  MissionHandler(const Mission& mission)
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
  std::unique_ptr<MoveBaseClient> action_client_;

  // Index of current waypoint
  unsigned int current_waypoint_index_;

  // Functions
  void SendGoal(void);
  move_base_msgs::MoveBaseGoal CreateCurrentGoal(void) const;
  void WaypointCallback(const actionlib::SimpleClientGoalState& state,
                        const ResultConstPtr& result);
};

}  // namespace propbot_mission
