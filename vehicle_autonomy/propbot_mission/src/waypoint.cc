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
    : gps_waypoint_(gps_waypoint), utm_zone_(utm_zone) {}

/**
 * Map waypoint accessor
 *
 * Returns waypoint as may waypoint with correct timestamp. The function uses
 * the utm waypoint and a ros transform to convert to a map waypoint.
 *
 * @return map_waypoint map waypoint with correct timestamp
 *
 */
geometry_msgs::PointStamped Waypoint::TransformToFrame(
    const std::string& frame_id) const {

  // Declare utm x and y variables
  double utm_northing = 0, utm_easting = 0;
  std::string utm_zone = utm_zone_;

  // Convert latitude and longitude to utm waypoints
  RobotLocalization::NavsatConversions::LLtoUTM(
      gps_waypoint_.first, gps_waypoint_.second, utm_northing, utm_easting,
      utm_zone);

  // Create utm waypoint based on conversion
  geometry_msgs::PointStamped utm_waypoint;
  utm_waypoint.header.frame_id = "utm";
  utm_waypoint.header.stamp = ros::Time(0);
  utm_waypoint.point.x = utm_easting;
  utm_waypoint.point.y = utm_northing;
  utm_waypoint.point.z = 0;

  // Declare transformed waypoint
  geometry_msgs::PointStamped transformed_waypoint;
  tf::TransformListener transform_listener;
  ros::Time time = ros::Time::now();

  bool transform_finished = false;
  while (!transform_finished) {
    try {
      // Wait for transform and then transform point
      utm_waypoint.header.stamp = ros::Time::now();
      transform_listener.waitForTransform(frame_id, "utm", time,
                                          ros::Duration(3.0));
      transform_listener.transformPoint(frame_id, utm_waypoint,
                                        transformed_waypoint);
      transform_finished = true;
    } catch (tf::TransformException& exception) {
      ROS_WARN("%s", exception.what());
      ros::Duration(0.01).sleep();
    }
  }

  ROS_INFO("Map waypoint is %f , %f.", transformed_waypoint.point.x,
           transformed_waypoint.point.y);
  return transformed_waypoint;
}
