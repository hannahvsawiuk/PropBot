#include <string>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

/* Publish slam pose node
 *
 * This node reads /tf and publishes pose with constant covariance values.
 * 
 */
int main(int argc, char** argv) {
  // Initiation node called send mission
  ros::init(argc, argv, "publish_slam_pose");
  ros::NodeHandle node_handle;
  
  // Get transform frame id
  std::string frame_id_param = "/" + ros::this_node::getName() + "/frame_id";
  std::string frame_id;
  ros::param::get(frame_id_param, frame_id);
  
  ROS_INFO("Publishing poses in %s frame", frame_id.c_str());

  // Create publisher
  ros::Publisher pose_pub = 
      node_handle.advertise<geometry_msgs::PoseWithCovarianceStamped>(
          "/slam/pose", 100);

  ros::Rate loop_rate(50);

  // Declare transform listener
  tf::TransformListener transform_listener;
  while (node_handle.ok()) {

    // Declare robot pose (all zeros) at origin of base link
    geometry_msgs::PoseStamped pose_base_link;
    pose_base_link.header.frame_id = "base_link";
    pose_base_link.pose.position.x = 0;
    pose_base_link.pose.position.y = 0;
    pose_base_link.pose.position.z = 0;
    pose_base_link.pose.orientation.w = 1.0;
    pose_base_link.pose.orientation.x = 0;
    pose_base_link.pose.orientation.y = 0;
    pose_base_link.pose.orientation.z = 0;

    // Declare target pose and stamped transform
    geometry_msgs::PoseStamped pose;
    tf::StampedTransform transform;

    try {
      // Transform to desired frame_id
      transform_listener.lookupTransform(frame_id, "base_link", ros::Time(0),
                                         transform);
      pose_base_link.header.stamp = transform.stamp_;
      transform_listener.transformPose(frame_id, pose_base_link, pose);
    } catch (tf::TransformException& ex) {
      ROS_ERROR("%s", ex.what());
    }

    geometry_msgs::PoseWithCovarianceStamped pose_with_covariance;
    pose_with_covariance.header = std::move(pose.header);
    pose_with_covariance.pose.pose = std::move(pose.pose);
    pose_with_covariance.pose.covariance = {
        0.001, 0, 0,     0, 0,     0, 0, 0.001, 0, 0,     0, 0,
        0,     0, 0.001, 0, 0,     0, 0, 0,     0, 0.001, 0, 0,
        0,     0, 0,     0, 0.001, 0, 0, 0,     0, 0,     0, 0.001};

    // Publish pose
    pose_pub.publish(pose_with_covariance);

    // Sleep
    loop_rate.sleep();
  }
  ros::shutdown();
  return 0;
}
