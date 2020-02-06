#pragma once

#include <geometry_msgs/PointStamped.h>

namespace propbot {

/**
 * Waypoint container
 *
 * This class stores waypoints and manages conversions between different
 * types.
 *
 */
class Waypoint {
 public:
    /* Delete default constructor */
  Waypoint() = delete;
  Waypoint(std::pair<double, double> gps_waypoint, std::string utm_zone);
  
  /* Gps waypoint accessor */
  std::pair<double, double> gps_waypoint() const { return gps_waypoint_; }
  geometry_msgs::PointStamped map_waypoint();

 private:
  // Waypoint in map odom frame
  std::pair<double, double> gps_waypoint_;

  std::string utm_zone_;
  geometry_msgs::PointStamped utm_waypoint_;
};

}  // namespace propbot