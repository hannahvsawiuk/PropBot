#pragma once

#include <atomic>
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
  /* Delete default constructor */
  MissionHandler() = delete;
  /* Mission handler constructor with a mission as an input */
  MissionHandler(const Mission& mission)
      : mission_(mission), current_waypoint_index_(0), is_finished_(false){};

  // Mission commands
  void Start();
  void Pause();
  void End();
  /* Function that returns if the mission is finished */
  bool IsFinished() const { return is_finished_; }

  // Accessors
  Waypoint current_waypoint() const;
  unsigned int current_waypoint_number() const;

 private:
  // Mission
  Mission mission_;

  // Action client
  std::unique_ptr<MoveBaseClient> action_client_;

  // Index of current waypoint
  unsigned int current_waypoint_index_;

  // Mission finished flag
  std::atomic<bool> is_finished_;

  // Functions
  void SendGoal();
  void SetDesiredOrientation(const Waypoint& current_waypoint,
                             const Waypoint& next_waypoint,
                             move_base_msgs::MoveBaseGoal* current_goal) const;
  move_base_msgs::MoveBaseGoal CreateCurrentGoal() const;
  void WaypointCallback(const actionlib::SimpleClientGoalState& state,
                        const ResultConstPtr& result);
};

}  // namespace propbot_mission
