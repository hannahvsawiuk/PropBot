#include <propbot_mission/mission_handler.hh>

#include <cassert>
#include <chrono>
#include <cmath>
#include <functional>

#include <tf/tf.h>

#include <propbot_util/exception.hh>

using namespace propbot;

/**
 * Start mission function
 *
 * This function starts the mission by sending the waypoints to the robot
 *
 */

void MissionHandler::Start() {
  // Start mision
  finished_ = false;
  // Declare an action client that communicates with move_base
  action_client_ = std::make_unique<MoveBaseClient>("/move_base", true);
  auto end =
      std::chrono::high_resolution_clock::now() + std::chrono::seconds(60);
  // Wait for action server to come up
  ROS_INFO("Waiting for move_base action server to come up");

  while (!action_client_->waitForServer(ros::Duration(15.0))) {
    if (std::chrono::high_resolution_clock::now() >= end) {
      throw Exception("Move_base action server did not come up!");
    }
  }
  // Send the first waypoint as a goal
  SendGoal();
}

/**
 * Pause mission function
 *
 * This function pauses the mission by cancelling all of the current goals in
 * the action server. However, it retains the current waypoint index, so when
 * the mission is started again, it resumes from where it was stopped.
 *
 */
void MissionHandler::Pause() {
  assert(action_client_);
  action_client_->cancelAllGoals();
}

/**
 * Stop mission function
 *
 * This function stops the mission by cancelling all of the goals in the action
 * server. It also sets the mission finished flag to true.
 *
 */
void MissionHandler::End() {
  assert(action_client_);
  action_client_->cancelAllGoals();
  finished_ = true;
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
void MissionHandler::SendGoal() {
  assert(action_client_);
  ROS_INFO("Sending waypoint number %i...", current_waypoint_number());

  // Create a lambda for the SimpleDoneCallback
  // Note: A lambda was used because of compiler issues when using std::bind
  // with WaypointCallback.
  auto waypoint_cb = [this](const actionlib::SimpleClientGoalState& state,
                            const ResultConstPtr& result) {
    this->WaypointCallback(state, result);
  };
  auto current_map_waypoint = current_waypoint().TransformToFrame("map");
  auto next_map_waypoint =
      mission_.mission()[current_waypoint_index_ + 1].TransformToFrame("map");

  action_client_->sendGoal(
      CreateCurrentGoal(current_map_waypoint, next_map_waypoint), waypoint_cb);
}

/**
 * Calculate desired orientation function.
 *
 * This function uses the current goal waypoint along with the next goal
 * waypoint to calculate the orientation of the current waypoint. It calculates
 * the desired current orientation such that the robot is pointing in the
 * direction of the next waypoint.
 *
 * @param current_waypoint Current may waypoint to be used for orientation goal.
 * @param next_waypoint Next waypoint to be used for orientation goal.
 * @param current_goal Current MoveBaseGoal to fill in with desired orientation
 * goal.
 *
 */

void MissionHandler::SetDesiredOrientation(
    const geometry_msgs::PointStamped& current_map_waypoint,
    const geometry_msgs::PointStamped& next_map_waypoint,
    move_base_msgs::MoveBaseGoal* current_goal) const {
  // Find difference between x and y components of the waypoints
  float delta_x = current_map_waypoint.point.x - next_map_waypoint.point.x;
  float delta_y = current_map_waypoint.point.y - next_map_waypoint.point.y;

  // Calculate the required yaw movement
  float yaw = atan2(delta_y, delta_x);

  // Calculate Euler rotation with pitch, roll = 0
  tf::Matrix3x3 euler_ypr_rot;
  euler_ypr_rot.setEulerYPR(yaw, 0, 0);

  // Convert to Quarternion
  tf::Quaternion quaternion_rot;
  euler_ypr_rot.getRotation(quaternion_rot);

  // Set goal orientation
  current_goal->target_pose.pose.orientation.x = quaternion_rot.getX();
  current_goal->target_pose.pose.orientation.y = quaternion_rot.getY();
  current_goal->target_pose.pose.orientation.z = quaternion_rot.getZ();
  current_goal->target_pose.pose.orientation.w = quaternion_rot.getW();
}

/**
 * Create goal function
 *
 * This function creates a move base goal based on the current waypoint.
 *
 */
move_base_msgs::MoveBaseGoal MissionHandler::CreateCurrentGoal(
    const geometry_msgs::PointStamped& current_map_waypoint,
    const geometry_msgs::PointStamped& next_map_waypoint) const {
  // Declare a move base goal
  move_base_msgs::MoveBaseGoal current_goal;

  // Set current_goal frame
  current_goal.target_pose.header.frame_id = "odom";
  current_goal.target_pose.header.stamp = ros::Time::now();

  Waypoint curr_waypoint = current_waypoint();
  // Set x and y of current_goal
  current_goal.target_pose.pose.position.x = current_map_waypoint.point.x;
  current_goal.target_pose.pose.position.y = current_map_waypoint.point.y;

  if (current_waypoint_number() < mission_.number_waypoints()) {
    // Calculate goal orientation using current and next waypoint
    SetDesiredOrientation(current_map_waypoint, next_map_waypoint,
                          &current_goal);
  } else {
    // Current waypoint is last waypoint, set orientation of current goal to a 0
    // rotation angle around an axis v(0,0,0)
    current_goal.target_pose.pose.orientation.w = 1.0;
  }

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
    const ResultConstPtr& result) {
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Robot has reached waypoint number %i", current_waypoint_number());
    if (current_waypoint_number() < mission_.number_waypoints()) {
      // Last waypoint has not been reached, send next waypoint
      current_waypoint_index_++;
      SendGoal();
    } else {
      // Last waypoint has been reached, set mission to finished
      finished_ = true;
      ROS_INFO("Mission is finished. ");
    }
  } else {
    ROS_ERROR("Robot was unable to reach waypoint number %i",
              current_waypoint_number());
    ROS_INFO("Stopping mission...");
    failed_ = true;
    End();
  }
}