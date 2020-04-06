#include <string>

#include <ros/ros.h>
#include <ros/package.h>

#include <people_msgs/People.h>
#include <people_msgs/Person.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <darknet_ros_msgs/BoundingBoxes.h>

#include <propbot_util/ros.hh>

#include <bounding_box_transformer.hh>

#include <unordered_map>

/**
 * Publish People
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "publish_people_node");
  ros::NodeHandle node_handle("publish_people_node");

  // Camera Topic
  const std::string camera_topic = propbot::getParamOrDefault(node_handle,
      "camera_topic", std::string("/camera/rgb/image"));

  // Camera Frame Rate
  const double camera_frame_rate = propbot::getParamOrDefault(node_handle,
      "camera_frame_rate", 30.0);

  if (camera_frame_rate <= 0.0) {
    ROS_ERROR("camera_frame_rate must be greater than 0");
    return 1;
  }

  ROS_INFO_STREAM("Receiving Images from Topic: " << camera_topic <<
      ", expecting " << camera_frame_rate << " images/s");

  // People Topic
  const std::string people_topic = propbot::getParamOrDefault(node_handle,
      "people_topic", std::string("/people"));

  // People Frame
  const std::string people_frame = propbot::getParamOrDefault(node_handle,
      "people_frame", std::string("map"));

  ROS_INFO_STREAM("Publishing People to Topic: " << people_topic <<
      ", in frame: " << people_frame);

  // Bounding Boxes Topic
  const std::string bb_topic = propbot::getParamOrDefault(node_handle,
      "bb_topic", std::string("/darknet_ros/bounding_boxes"));

  ROS_INFO_STREAM("Receiving Bounding Boxes from Topic: " << bb_topic);

  // Marker Topic
  const std::string marker_topic = propbot::getParamOrDefault(node_handle,
      "marker_topic", std::string("/propbot_object_detection/markers"));

  // Laser Scan Topic
  const std::string laser_topic = propbot::getParamOrDefault(node_handle,
      "laser_topic", std::string("/scan"));

  // Ray length
  const double ray_length = propbot::getParamOrDefault(node_handle,
      "ray_length", 1000.0);

  if (ray_length <= 0.0) {
    ROS_ERROR("ray_length must be greater than 0");
    return 1;
  }

  // Minimum Probability
  const double min_probability = propbot::getParamOrDefault(node_handle,
      "min_probability", 0.5);

  if (min_probability < 0.0 || 1.0 < min_probability) {
    ROS_ERROR("min_probability must be within [0.0, 1.0]");
    return 1;
  }

  // Filter Ids
  const std::vector<std::string> filter_classes =
    propbot::getParamOrDefault(node_handle,
        "filter_classes", std::vector<std::string>{"person"});

  if (filter_classes.empty()) {
    ROS_ERROR("Filter IDs is Empty, no work to do!");
    return 1;
  }

  ROS_INFO_STREAM("Receiving Laser Scans from Topic: " << laser_topic);

  ros::Publisher people_pub = node_handle.advertise<people_msgs::People>(
      people_topic, 10);

  ros::Publisher marker_pub = node_handle.advertise<visualization_msgs::MarkerArray>(
      marker_topic, 10);

  std::unordered_map<int, BoundingCylinder> allBBs;
  auto bbCallback = [&](std::vector<BoundingCylinder> cylinders) {
    visualization_msgs::MarkerArray marker_arr;

    ros::Time now = ros::Time::now();
    for (auto& cylinder : cylinders) {
      visualization_msgs::Marker marker;
      marker.header.frame_id = people_frame;
      marker.header.stamp = now;
      marker.ns = "propbot_object_detection";
      marker.id = cylinder.id;
      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = cylinder.center.x();
      marker.pose.position.y = cylinder.center.y();
      marker.pose.position.z = cylinder.center.z();
      marker.pose.orientation.x = 0;
      marker.pose.orientation.y = 0;
      marker.pose.orientation.z = 0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = cylinder.radius * 2;
      marker.scale.y = cylinder.radius * 2;
      marker.scale.z = cylinder.height;
      marker.color.r = 0;
      marker.color.g = 1.0;
      marker.color.b = 0;
      marker.color.a = 0.5;
      marker_arr.markers.push_back(marker);

      allBBs.insert_or_assign(cylinder.id, std::move(cylinder));
    }

    for (auto it = allBBs.begin(); it != allBBs.end();) {
      if (now - it->second.creationTime > ros::Duration(1.0)) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = people_frame;
        marker.header.stamp = now;
        marker.ns = "propbot_object_detection";
        marker.id = it->second.id;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::DELETE;
        marker_arr.markers.push_back(marker);

        it = allBBs.erase(it);
      } else {
        it++;
      }
    }

    marker_pub.publish(marker_arr);
  };

  BoundingBoxTransformer bbTransformer(people_frame, bbCallback, filter_classes,
      min_probability, ray_length, ros::Duration(1.0 / camera_frame_rate));

  image_transport::ImageTransport image_transport(node_handle);

  image_transport::CameraSubscriber cam_sub = image_transport.subscribeCamera(
      camera_topic, 1 /* Queue Size */,
      &BoundingBoxTransformer::handleCameraImage,
      &bbTransformer);

  ros::Subscriber bb_sub = node_handle.subscribe(
      bb_topic, 1 /* Queue Size */,
      &BoundingBoxTransformer::handleBoundingBoxes,
      &bbTransformer);

  ros::Subscriber laser_sub = node_handle.subscribe(
      laser_topic, 1 /* Queue Size */,
      &BoundingBoxTransformer::handleLaserScan,
      &bbTransformer);

  ros::spin();
  ros::shutdown();
  return 0;
}
