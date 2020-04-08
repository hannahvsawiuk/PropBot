#include <bounding_box_transformer.hh>

#include <bounding_cylinder.hh>

#include <darknet_ros_msgs/BoundingBox.h>

#include <vector>
#include <algorithm>
#include <numeric>
#include <utility>
#include <limits>
#include <cassert>
#include <cmath>

namespace
{
float planeAngleFromXAxisSigned(tf::Vector3 vector)
{
    tf::Vector3 v(vector.x(), vector.y(), 0.0);

    if (v.y() >= 0.0)
        return v.angle(tf::Vector3(1.0, 0.0, 0.0));
    else
        return -v.angle(tf::Vector3(1.0, 0.0, 0.0));
}

std::pair<float, float> findLaserDepthMinMax(
        const sensor_msgs::LaserScanConstPtr& laser,
        float angle_left, float angle_right)
{
    //ROS_INFO_STREAM("Find Angles: " << angle_left << ", " << angle_right <<
    //        " from " << laser->angle_min << ", " << laser->angle_max);

    float left_from_min = angle_left - laser->angle_min;
    if (left_from_min < 0.0)
        left_from_min = 0.0;
    if (left_from_min > (laser->angle_max - laser->angle_min))
        left_from_min = (laser->angle_max - laser->angle_min);

    float right_from_min = angle_right - laser->angle_min;
    if (right_from_min < 0.0)
        right_from_min = 0.0;
    if (right_from_min > (laser->angle_max - laser->angle_min))
        right_from_min = (laser->angle_max - laser->angle_min);

    size_t s_idx = std::floor(left_from_min / laser->angle_increment);
    size_t e_idx = std::ceil(right_from_min / laser->angle_increment) + 1;

    if (s_idx >= laser->ranges.size())
        s_idx -= laser->ranges.size();

    if (e_idx >= laser->ranges.size())
        e_idx -= laser->ranges.size();

    assert(s_idx < laser->ranges.size());
    assert(e_idx < laser->ranges.size());

    std::vector<float> found_ranges;

    float sum = 0.0;
    float min = std::numeric_limits<float>::max();
    float max = 0.0;
    size_t count = 0;
    for (size_t i = s_idx; i != e_idx; i = (i < laser->ranges.size() - 1) ? i + 1 : 0) {
        float range = laser->ranges[i];
        if (laser->range_min <= range && range < laser->range_max) {
            sum += range;
            count++;
            found_ranges.push_back(range);

            if (range < min)
                min = range;

            if (range > max)
                max = range;
        }
    }

    std::sort(found_ranges.begin(), found_ranges.end());

    if (found_ranges.size() > 0) {
        std::vector<int> groups;
        groups.push_back(0);

        // Group nearby depths together since they are likely
        // to be from the same object
        for (size_t i = 1; i < found_ranges.size(); i++) {
            if (found_ranges[i] - found_ranges[groups.back()] > 1.0)
                groups.push_back(i);
        }

        if (groups.size() >= 2) {
        //if (groups.size() >= 3) {
        //    // Some foreground obstruction then target then background
        //    // this may not work correctly when there are multiple obstructions.
        //    return {found_ranges[groups[1]], found_ranges[groups[2] - 1]};
        //} else if (groups.size() == 2) {
            // Target is in the foreground, remove the background
            return {found_ranges.front(), found_ranges[groups[1] - 1]};
        } else {
            // Target is the only thing seen (not too likely)
            return {found_ranges.front(), found_ranges.back()};
        }
    } else {
        return {-1.0, -1.0};
    }
}
}

void BoundingBoxTransformer::handleCameraImage(
        const sensor_msgs::ImageConstPtr& image_msg,
        const sensor_msgs::CameraInfoConstPtr& cam_info_msg)
{
    last_image_time_ = image_msg->header.stamp;

    // Update our camera model from this image
    if (!pinhole_camera_.initialized())
        pinhole_camera_.fromCameraInfo(cam_info_msg);
}

void BoundingBoxTransformer::handleBoundingBoxes(
        const darknet_ros_msgs::BoundingBoxesConstPtr& bb_msg)
{
    if (!pinhole_camera_.initialized()) {
        ROS_WARN_STREAM_THROTTLE(1, "Waiting for Camera Initialization");
        return;
    }

    if (!camera_to_laser_) {
        ROS_WARN_STREAM_THROTTLE(1, "Waiting for Laser Transform");
    }

    auto opt_laser = findLaserScanForTime(last_image_time_);
    if (!opt_laser) {
        ROS_WARN_STREAM_THROTTLE(1, "Waiting for Laser Scan");
        return;
    }

    const auto& laser = *opt_laser;

    // Find a 3D point (in the laser frame) corresponding to the pixel
    // this is done by extending the ray from camera to pixel out to ray length
    auto projectPt = [&](int x, int y) -> tf::Point {
        auto cvRay = pinhole_camera_.projectPixelTo3dRay(cv::Point2d(x, y));

        // Rays originate from the camera origin, no offset needed
        tf::Point pt = tf::Vector3(cvRay.x, cvRay.y, cvRay.z).normalized() * ray_length_;

        tf::Point transformedPt = *camera_to_laser_ * pt;

        return transformedPt;
    };

    std::vector<BoundingCylinder> bbCylinders;
    for (const auto& bb : bb_msg->bounding_boxes) {
        if (bb.probability < min_prob_ || classes_.count(bb.Class) != 1)
            continue;

        tf::Point topLeft = projectPt(bb.xmin, bb.ymin);
        tf::Point bottomLeft = projectPt(bb.xmin, bb.ymax);
        tf::Point topRight = projectPt(bb.xmax, bb.ymin);
        tf::Point bottomRight = projectPt(bb.xmax, bb.ymax);

        // Laser angle from X axis, note that angles to the right are negative
        float topLeftAngle = planeAngleFromXAxisSigned(topLeft);
        float bottomLeftAngle = planeAngleFromXAxisSigned(bottomLeft);
        float topRightAngle = planeAngleFromXAxisSigned(topRight);
        float bottomRightAngle = planeAngleFromXAxisSigned(bottomRight);

        // Use the most left angle
        tf::Vector3 left;
        float leftAngle;
        if (topLeftAngle > bottomLeftAngle) {
            left = {topLeft.x(), topLeft.y(), 0};
            leftAngle = topLeftAngle;
        } else {
            left = {bottomLeft.x(), bottomLeft.y(), 0};
            leftAngle = bottomLeftAngle;
        }
        left.normalize();

        // Use the most right angle
        tf::Vector3 right;
        float rightAngle;
        if (topRightAngle < bottomRightAngle) {
            right = {topRight.x(), topRight.y(), 0};
            rightAngle = topRightAngle;
        } else {
            right = {bottomRight.x(), bottomRight.y(), 0};
            rightAngle = bottomRightAngle;
        }
        right.normalize();

        auto [min, max] = findLaserDepthMinMax(laser, leftAngle - 0.2, rightAngle + 0.2);
        if (min < 0.0 || max < 0.0) {
            ROS_WARN_STREAM_THROTTLE(1.0, "Could not find depth for " <<
                    bb.Class << " " << bb.id);
            continue;
        }

        tfScalar avg = (max + min) / 2.0;

        left *= avg;
        right *= avg;

        auto topZ = std::max((topLeft.normalized() * avg).z(), (topRight.normalized() * avg).z());
        auto bottomZ = std::min((bottomLeft.normalized() * avg).z(), (bottomRight.normalized() * avg).z());
        auto middleZ = (topZ - bottomZ) / 2.0;
        auto height = topZ - bottomZ;

        auto diameter = std::max<tfScalar>(max - min, left.distance(right));
        tf::Vector3 center = (right - left) / 2 + left + tf::Vector3(0, 0, middleZ);

        bbCylinders.emplace_back(bb.Class, bb.id, bb_msg->header.stamp,
                center, diameter / 2, height);
    }

    tf::StampedTransform laser_to_person;
    try {
        tf_listener_.lookupTransform(people_frame_, laser->header.frame_id,
                last_image_time_, laser_to_person);
    } catch (const tf::TransformException& ex) {
        ROS_WARN_STREAM("Failed to transform from laser to person: " << ex.what());
        try {
            tf_listener_.lookupTransform(people_frame_, laser->header.frame_id,
                    ros::Time(0), laser_to_person);
        } catch (const tf::TransformException& ex) {
            ROS_WARN_STREAM("Failed to transform from laser to person: " << ex.what());
            return;
        }
    }

    for (auto& bbCylinder : bbCylinders)  {
        bbCylinder.center = laser_to_person * bbCylinder.center;
    }

    callback_(bbCylinders);
}

void BoundingBoxTransformer::handleLaserScan(
        const sensor_msgs::LaserScanConstPtr& laser_msg)
{
    if (!pinhole_camera_.initialized()) {
        ROS_WARN_STREAM_THROTTLE(1, "Waiting for Camera Initialization");
        return;
    }

    // This transform should be constant since they are both mounted to the robot
    if (!camera_to_laser_) {
        tf::StampedTransform cam_to_laser;
        try {
            tf_listener_.lookupTransform(laser_msg->header.frame_id,
                    pinhole_camera_.tfFrame(), ros::Time(0), cam_to_laser);
            camera_to_laser_ = cam_to_laser;
        } catch (const tf::TransformException& ex) {
            ROS_WARN_STREAM("Failed to transform from camera to laser: " << ex.what());
        }
    }

    if (last_lasers_.size() >= NUM_LASER_SCANS)
        last_lasers_.pop_back();

    last_lasers_.push_front(laser_msg);
}

std::optional<sensor_msgs::LaserScanConstPtr> BoundingBoxTransformer::findLaserScanForTime(ros::Time t)
{
    double timeout = tf_timeout_.toSec();
    double bestDiff = std::numeric_limits<double>::max();
    const sensor_msgs::LaserScanConstPtr* bestScan = nullptr;
    for (const auto& laser : last_lasers_) {
        double diff = std::abs((laser->header.stamp - t).toSec());
        if (diff < bestDiff) {
            bestDiff = diff;
            bestScan = &laser;
        }
    }

    if (bestScan != nullptr && bestDiff < timeout)
        return *bestScan;
    else
        return {};
}

