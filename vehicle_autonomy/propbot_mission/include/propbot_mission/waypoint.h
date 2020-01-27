#ifndef PROPBOT_MISSION_WAYPOINT_H_
#define PROPBOT_WAYPOINT_H_

#include <geometry_msgs/PointStamped.h>


namespace propbot_mission {

/**
 * Waypoint container
 *
 * This class stores waypoints and manages conversions between different
 * types.
 *
 */
class Waypoint {
 public:
    /* Default constructor */
  Waypoint() = default;
  Waypoint(const double x, const double y);

  
  geometry_msgs::PointStamped map_waypoint() const;

 private:
  // Waypoint in map odom frame
  const geometry_msgs::PointStamped map_waypoint_;
};

}  // namespace propbot_mission

#endif  // PROPBOT_WAYPOINT_H_