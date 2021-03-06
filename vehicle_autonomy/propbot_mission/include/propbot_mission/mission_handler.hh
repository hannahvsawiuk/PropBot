#pragma once

#include <atomic>
#include <memory>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>

#include <propbot_mission/mission.hh>
#include <propbot_mission/waypoint.hh>

namespace propbot {

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
      : mission_(mission),
        current_waypoint_index_(0),
        finished_(false),
        failed_(false){};

  // Mission commands
  void Start();
  void Pause();
  void Resume();
  void End();
  /* Function that returns if the mission is finished */
  bool Finished() const { return finished_; }
  /* Function that returns if the mission failed */
  bool Failed() const { return failed_; }

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
  std::atomic_bool finished_;
  std::atomic_bool failed_;

  // Functions
  void SendGoal();
  move_base_msgs::MoveBaseGoal CreateCurrentGoal() const;
  void WaypointCallback(const actionlib::SimpleClientGoalState& state,
                        const ResultConstPtr& result);
};

}  // namespace propbot
