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
 * @param utm_zone UTM zone of the mission
 *
 */
Mission::Mission(const std::string &mission_file, const std::string &utm_zone) {
  std::ifstream file(mission_file);

  double latitude, longitude; 
  if (file.is_open()) {
    while (!file.eof()) {
      file >> latitude;
      file >> longitude;

      mission_.push_back(Waypoint(std::pair<double, double>(latitude, longitude), utm_zone));
    }
    size_ = mission_.size();
    ROS_INFO("%i Map waypoints were read", size_);
    file.close();
  } else {
    ROS_ERROR("Unable to open waypoint file");
  }
}
