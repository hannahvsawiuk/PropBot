#include <propbot_mission/mission_handler.h>

using namespace propbot_mission;

/**
 * Start mission function
 *
 * This function starts the mission by sending the waypoints to the robot
 *
 */

Mission::Start() {
  // Declare an action client that communicates with move_base
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client(
      "/move_base", true);

  // Wait for action server to come up
  ROS_INFO("Waiting for move_base action server to come up");
  if (!action_client.waitForServer(ros::Duration(15.0))) {
    ROS_ERROR("Move_base action server did not come up!");
    
  }

  auto waypoint_cb = boost::bind(&Mission::WaypointCallback, this);
  action_client.sendGoal(CreateCurrentGoal(), waypoint_cb);
}

/* Current waypoint accessor */
Waypoint current_waypoint() const {
  return mission_.mission()[current_waypoint_index_];
};

/* Current waypoint number accessor */
unsigned int current_waypoint_number() const {
  return current_waypoint_index_ + 1;
};

/**
 * Create goal function
 *
 * This function creates a move base goal based on the current waypoint
 *
 */
move_base_msgs::MoveBaseGoal Mission::CreateCurrentGoal() {
  // Declare a move base goal
  move_base_msgs::MoveBaseGoal current_goal;

  // Set current_goal frame
  current_goal.target_pose.header.frame_id = "odom";
  current_goal.target_pose.header.stamp = ros::Time::now();

  // Set x and y of current_goal
  current_goal.target_pose.pose.position.x = current_waypoint().map_waypoint.point.x;
  current_goal.target_pose.pose.position.y = current_waypoint().map_waypoint.point.y;

  // Set orientation of current goal to a 0 rotation angle around an axis v(0,0,0)
  current_goal.target_pose.pose.orientation.w = 1.0;

  return current_goal;
}

Mission::WaypointCallback(const actionlib::SimpleClientGoalState& state,
                          const actionlib::ResultConstPtr& result) {}