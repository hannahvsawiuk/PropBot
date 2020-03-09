#include <propbot_mission/mission.hh>

#include <propbot_util/exception.hh>

using namespace propbot;

/**
 * Mission constructor
 *
 * @brief This is a mission constructor that takes in a text file of x and y map
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

  // Create a vector of waypoints using the latitude and longitude
  double latitude, longitude; 
  if (file.is_open()) {
    while (!file.eof()) {
      file >> latitude;
      file >> longitude;

      mission_.push_back(Waypoint({latitude, longitude}, utm_zone));
    }
    ROS_INFO("%lu Map waypoints were read", mission_.size());
    file.close();
  } else {
    throw Exception("Unable to open waypoint file");
  }
}
