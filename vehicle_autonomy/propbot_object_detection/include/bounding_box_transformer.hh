#pragma once

#include <ros/ros.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>

#include <darknet_ros_msgs/BoundingBoxes.h>

#include <tf/transform_listener.h>

#include <image_transport/image_transport.h>

#include <image_geometry/pinhole_camera_model.h>

#include <bounding_cylinder.hh>

#include <string>
#include <unordered_set>
#include <utility>
#include <functional>
#include <deque>

class BoundingBoxTransformer
{
public:
  using BBCallback = std::function<void(std::vector<BoundingCylinder>)>;

  BoundingBoxTransformer(std::string people_frame,
      BBCallback cb,
      std::vector<std::string> classes,
      double min_probability,
      double ray_length,
      ros::Duration tf_timeout)
    : people_frame_(people_frame)
    , callback_(cb)
    , classes_(classes.begin(), classes.end())
    , min_prob_(min_probability)
    , ray_length_(ray_length)
    , tf_timeout_(tf_timeout)
  {}

  void handleCameraImage(const sensor_msgs::ImageConstPtr& image_msg,
      const sensor_msgs::CameraInfoConstPtr& cam_info_msg);

  void handleBoundingBoxes(const darknet_ros_msgs::BoundingBoxesConstPtr& bb_msg);

  void handleLaserScan(const sensor_msgs::LaserScanConstPtr& laser_msg);
private:
  const std::string people_frame_;
  const BBCallback callback_;
  const std::unordered_set<std::string> classes_;
  const double min_prob_;
  const double ray_length_;
  const ros::Duration tf_timeout_;
  image_geometry::PinholeCameraModel pinhole_camera_;
  tf::TransformListener tf_listener_;
  std::deque<sensor_msgs::LaserScanConstPtr> last_lasers_;

  static constexpr int NUM_LASER_SCANS = 32;

  std::optional<sensor_msgs::LaserScanConstPtr> findLaserScanForTime(ros::Time t);
};
