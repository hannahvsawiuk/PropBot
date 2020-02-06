#pragma once

#include <fstream>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <propbot_mission/waypoint.hh>

namespace propbot {

/**
 * Mission container
 *
 * This class reads in and stores a mission.
 *
 */
class Mission {
 public:
  /* Delete default constructor */
  Mission() = delete;
  Mission(const std::string &mission_file, const std::string &utm_zone);

  /* Mission accessor */
  const std::vector<Waypoint>& mission() const { return mission_; };

  /* Mission number of waypoints accessor */
  unsigned int number_waypoints() const { return mission_.size(); };

 private:
  // Mission waypoint vector
  std::vector<Waypoint> mission_;
};

}  // namespace propbot