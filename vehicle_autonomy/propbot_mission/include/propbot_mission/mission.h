#pragma once

#include <fstream>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <propbot_mission/waypoint.h>

namespace propbot_mission {

/**
 * Mission container
 *
 * This class reads in and stores a mission.
 *
 */
class Mission {
 public:
  /* Default constructor */
  Mission() = default;
  Mission(const std::string &mission_file, const std::string &utm_zone);

  /* Mission accessor */
  std::vector<Waypoint> mission() const { return mission_; };

  /* Mission size accessor */
  unsigned int size() const { return size_; };

 private:
  // Mission waypoint vector
  std::vector<Waypoint> mission_;

  // Size of mission
  unsigned int size_;
};

}  // namespace propbot_mission