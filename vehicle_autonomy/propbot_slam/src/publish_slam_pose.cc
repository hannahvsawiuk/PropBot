#include <string>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv) {
  // Initiation node called send mission
  ros::init(argc, argv, "publish_odometry");
  ros::NodeHandle node_handle;

  ros::Publisher slam_pose_pub =
      node_handle.advertise<geometry_msgs::PoseWithCovarianceStamped>(
          "propbot_slam/pose", 100);
  ros::Rate loop_rate(1);
  tf::TransformListener transform_listener;
  while (node_handle.ok()) {
    geometry_msgs::PoseStamped pose_base_link;
    pose_base_link.header.frame_id = "base_link";
    pose_base_link.pose.position.x = 0;
    pose_base_link.pose.position.y = 0;
    pose_base_link.pose.position.z = 0;
    pose_base_link.pose.orientation.w = 1.0;
    pose_base_link.pose.orientation.x = 0;
    pose_base_link.pose.orientation.y = 0;
    pose_base_link.pose.orientation.z = 0;

    geometry_msgs::PoseStamped pose_odom;
    // geometry_msgs::PoseStamped pose_odom;

    tf::StampedTransform transform;

    try {
      transform_listener.lookupTransform("odom", "base_link", ros::Time(0),
                                         transform);
      pose_base_link.header.stamp = transform.stamp_;
      transform_listener.transformPose("odom", pose_base_link, pose_odom);

    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }

    geometry_msgs::PoseWithCovarianceStamped pose_with_covariance_map;
    pose_with_covariance_map.header = std::move(pose_odom.header);
    pose_with_covariance_map.pose.pose = std::move(pose_odom.pose);
    pose_with_covariance_map.pose.covariance = {
        0.001, 0, 0,     0, 0,     0, 0, 0.001, 0, 0,     0, 0,
        0,     0, 0.001, 0, 0,     0, 0, 0,     0, 0.001, 0, 0,
        0,     0, 0,     0, 0.001, 0, 0, 0,     0, 0,     0, 0.001};

    slam_pose_pub.publish(pose_with_covariance_map);
    ros::spinOnce();
    loop_rate.sleep();
  }
  ros::shutdown();
  return 0;
}