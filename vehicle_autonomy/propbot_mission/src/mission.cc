#include <propbot_mission/mission.h>

using namespace propbot_mission;

/**
 * Mission constructor
 *
 * This is a mission constructor that takes in a text file of x and y map
 * waypoints.
 *
 * @param mission_file A text file with the waypoints. The x and y coordinates
 * are separated by spaces. The waypoints are separated by '\n'.
 *
 */
Mission::Mission(const std::string &mission_file) {
  std::ifstream file(mission_file);

  double x, y;
  if (file.is_open()) {
    while (!file.eof()) {
      file >> x;
      file >> y;

      mission_.push_back(Waypoint(x, y));
    }
    size_ = mission_.size();
    ROS_INFO("%i Map waypoints were read", size_);
    file.close();
  } else {
    ROS_ERROR("Unable to open waypoint file");
  }
}
