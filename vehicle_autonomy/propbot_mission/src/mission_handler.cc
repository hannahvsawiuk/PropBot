#include <propbot_mission/mission_handler.h>

using namespace propbot_mission;

/**
 * Start mission function
 *
 * This function starts the mission by sending the waypoints to the robot
 *
 */

void MissionHandler::Start(void) {
  // Declare an action client that communicates with move_base
  action_client_ = std::make_unique<MoveBaseClient>("/move_base", true);

  // Wait for action server to come up
  ROS_INFO("Waiting for move_base action server to come up");
  if (!action_client_->waitForServer(ros::Duration(15.0))) {
    ROS_ERROR("Move_base action server did not come up!");
  }
}

/**
 * Stop mission function
 *
 * This function stops the mission by cancelling all of the goals in the action
 * server. However, it retains the current waypoint index, so when the mission
 * is started again, it resumes from where it was stopped.
 *
 */
void MissionHandler::Stop(void) const {
  if (!action_client_) {
    action_client_->cancelAllGoals();
  } else {
    ROS_ERROR("Action client has not been started! Cannot stop mission.");
  }
}

/**
 * Current waypoint accessor
 *
 */
Waypoint MissionHandler::current_waypoint() const {
  return mission_.mission()[current_waypoint_index_];
};

/**
 * Current waypoint number accessor
 *
 */
unsigned int MissionHandler::current_waypoint_number() const {
  return current_waypoint_index_ + 1;
};

/**
 * Send goal function
 *
 * This function sends a goal based on the current waypoint to move base.
 *
 */
void MissionHandler::SendGoal(void) const {
  if (!action_client_) {
    ROS_INFO("Sending waypoint number %i...", current_waypoint_number());
    auto waypoint_cb = boost::bind(&MissionHandler::WaypointCallback, this);
    action_client_->sendGoal(CreateCurrentGoal(), waypoint_cb);
  } else {
    ROS_ERROR("Action client has not been started! Cannot send goal.");
  }
}

/**
 * Create goal function
 *
 * This function creates a move base goal based on the current waypoint.
 *
 */
move_base_msgs::MoveBaseGoal MissionHandler::CreateCurrentGoal(void) const {
  // Declare a move base goal
  move_base_msgs::MoveBaseGoal current_goal;

  // Set current_goal frame
  current_goal.target_pose.header.frame_id = "odom";
  current_goal.target_pose.header.stamp = ros::Time::now();

  // Set x and y of current_goal
  current_goal.target_pose.pose.position.x =
      current_waypoint().map_waypoint().point.x;
  current_goal.target_pose.pose.position.y =
      current_waypoint().map_waypoint().point.y;

  // Set orientation of current goal to a 0 rotation angle around an axis
  // v(0,0,0)
  current_goal.target_pose.pose.orientation.w = 1.0;

  return current_goal;
}

/**
 * Callback function for action client
 *
 * This function checks if the waypoint was reached and then sends the next
 * goal. If the waypoint was unreachable, it stops the mission.
 *
 * @param state Describes the state of the move base goal, including if the
 * robot suceeded in reaching the goal.
 * @param result Result of move base action
 *
 */
void MissionHandler::WaypointCallback(
    const actionlib::SimpleClientGoalState& state,
    const actionlib::ResultConstPtr& result) {
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Robot has reached waypoint number %i", current_waypoint_number());
    current_waypoint_index_++;
    SendGoal();
  } else {
    ROS_ERROR("Robot was unable to reach waypoint number %i",
              current_waypoint_number());
    ROS_INFO("Stopping mission...");
    Stop();
  }
}