#include <propbot_mission/waypoint.hh>

#include <robot_localization/navsat_conversions.h>
#include <tf/transform_listener.h>

using namespace propbot;

/**
 * Waypoint constructor
 *
 * @brief This is a waypoint constructor that takes in x and y coordinates in
 * the 'odom' frame of the map.
 *
 * @param gps_waypoint latitude, longitude pair og gps waypoint
 * @param utm_zone UTM zone of the waypoint
 *
 */
Waypoint::Waypoint(std::pair<double, double> gps_waypoint, std::string utm_zone)
    : gps_waypoint_(gps_waypoint), utm_zone_(utm_zone) {
  // Declare utm x and y variables
  double utm_northing = 0, utm_easting = 0;

  // Convert latitude and longitude to utm waypoints
  RobotLocalization::NavsatConversions::LLtoUTM(
      gps_waypoint_.first, gps_waypoint_.second, utm_northing, utm_easting,
      utm_zone_);

  utm_waypoint_.header.frame_id = "utm";
  utm_waypoint_.header.stamp = ros::Time(0);
  utm_waypoint_.point.x = utm_easting;
  utm_waypoint_.point.y = utm_northing;
  utm_waypoint_.point.z = 0;
}

/**
 * Map waypoint accessor
 *
 * Returns waypoint as may waypoint with correct timestamp. The function uses
 * the utm waypoint and a ros transform to convert to a map waypoint.
 *
 * @return map_waypoint map waypoint with correct timestamp
 *
 */
geometry_msgs::PointStamped Waypoint::map_waypoint() {
  geometry_msgs::PointStamped map_waypoint;
  map_waypoint.header.stamp = ros::Time(0);
  tf::TransformListener transform_listener;
  ros::Time time = ros::Time::now();
  // Try to transform point for a maximum of 3 times
  for (int i; i < 3; i++) {
    try {
      utm_waypoint_.header.stamp = ros::Time::now();
      transform_listener.waitForTransform("odom", "utm", time,
                                          ros::Duration(3.0));
      transform_listener.transformPoint("odom", utm_waypoint_, map_waypoint);
    } catch (tf::TransformException& exception) {
      ROS_WARN("%s", exception.what());
      ros::Duration(0.01).sleep();
    }
  }

  return map_waypoint;
}