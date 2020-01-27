#ifndef PROPBOT_MISSION_MISSION_
#define PROPBOT_MISSION_MISSION_

#include <fstream>
#include <string>
#include <vector>

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
  Mission(const std::string &mission_file);

  /* Mission accessor */
  std::vector<Waypoint::Waypoint> mission() const { return mission_; };

  /* Mission size accessor */
  unsigned int size() const { return size_; };

 private:
  // Mission waypoint vector
  std::vector<Waypoint::Waypoint> mission_;

  // Size of mission
  unsigned int size_;
};

}  // namespace propbot_mission

#endif  // PROPBOT_MISSION_MISSION_