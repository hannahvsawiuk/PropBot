#include <propbot_mission/waypoint.hh>

using namespace propbot;

/**
 * Waypoint constructor
 *
 * @brief This is a waypoint constructor that takes in x and y coordinates in
 * the 'odom' frame of the map.
 *
 * @param x x coordinate of waypoint in 'odom' frame
 * @param y y coordinate of waypoint in 'odom' frame
 *
 */
Waypoint::Waypoint(const double x, const double y) {
  map_waypoint_.header.frame_id = "odom";
  map_waypoint_.header.stamp = ros::Time(0);
  map_waypoint_.point.x = x;
  map_waypoint_.point.y = y;
  map_waypoint_.point.z = 0;
}

/**
 * Map waypoint accessor
 *
 * Returns waypoint accessor with correct timestamp.
 *
 * @return map_waypoint waypoint with correct timestamp
 *
 */
geometry_msgs::PointStamped Waypoint::map_waypoint() const {
  geometry_msgs::PointStamped map_waypoint = map_waypoint_;
  map_waypoint.header.stamp = ros::Time(0);
  return map_waypoint;
}