#include <bounding_box_transformer.hh>

#include <bounding_cylinder.hh>

#include <darknet_ros_msgs/BoundingBox.h>

#include <vector>
#include <algorithm>
#include <numeric>
#include <utility>
#include <limits>
#include <cassert>

namespace
{
std::pair<float, float> findLaserDepthMinMax(
        const sensor_msgs::LaserScanConstPtr& laser,
        float angle_min, float angle_max)
{
    std::vector<float> found_ranges;

    float sum = 0.0;
    float min = std::numeric_limits<float>::max();
    float max = 0.0;
    size_t count = 0;
    for (int i = 0; i < laser->ranges.size(); i++) {
        float range = laser->ranges[i];
        if (range < laser->range_min || laser->range_max < range)
            continue;

        float min = angle_min <= angle_max ? angle_min : angle_max;
        float max = angle_min <= angle_max ? angle_max : angle_min;

        float angle = laser->angle_min + i * laser->angle_increment;
        if ((angle_min <= angle_max) ? (min <= angle && angle <= max)
                : (min <= angle || max <= angle)) {
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
        int upper_idx;
        // FIXME: better heuristics for determining the min/max
        for(upper_idx = 0; upper_idx < found_ranges.size() - 1; upper_idx++) {
            if (found_ranges[upper_idx + 1] - found_ranges[0] > 1.0)
                break;
        }

        if (upper_idx != 0)
            return {found_ranges[0], found_ranges[upper_idx]};
        else
            return {found_ranges[0], found_ranges[0] + 1.0};
    } else {
        return {-1.0, -1.0};
    }
}
}

void BoundingBoxTransformer::handleCameraImage(
        const sensor_msgs::ImageConstPtr& image_msg,
        const sensor_msgs::CameraInfoConstPtr& cam_info_msg)
{
    // Update our camera model from this image
    pinhole_camera_.fromCameraInfo(cam_info_msg);
}

void BoundingBoxTransformer::handleBoundingBoxes(
        const darknet_ros_msgs::BoundingBoxesConstPtr& bb_msg)
{
    if (!pinhole_camera_.initialized()) {
        ROS_WARN_STREAM_THROTTLE(1, "Waiting for Camera Initialization");
        return;
    }

    auto opt_laser = findLaserScanForTime(pinhole_camera_.stamp());
    if (!opt_laser) {
        ROS_WARN_STREAM_THROTTLE(1, "Waiting for Laser Scan");
        return;
    }

    const auto& laser = *opt_laser;

    const std::string& laser_frame = laser->header.frame_id;
    try {
        // Find the transform
        tf_listener_.waitForTransform(laser_frame,
                pinhole_camera_.tfFrame(),
                pinhole_camera_.stamp(),
                tf_timeout_);
    } catch (const tf::TransformException& ex) {
        ROS_WARN_STREAM("Failed to get transform from camera to laser: " << ex.what());
        return;
    }

    auto& bb_vec = bb_msg->bounding_boxes;

    // Find a 3D point (in the laser frame) corresponding to the pixel
    // this is done by extending the ray from camera to pixel out to ray length
    auto projectPt = [&](int x, int y) -> tf::Vector3 {
        auto cvRay = pinhole_camera_.projectPixelTo3dRay(cv::Point2d(x, y));

        // Rays originate from the camera origin, no offset needed
        tf::Vector3 ptN(cvRay.x, cvRay.y, cvRay.z);
        ptN.normalize();

        geometry_msgs::PointStamped pt;

        pt.header.frame_id = pinhole_camera_.tfFrame();
        pt.header.stamp = pinhole_camera_.stamp();
        pt.point.x = ptN.x() * ray_length_;
        pt.point.y = ptN.y() * ray_length_;
        pt.point.z = ptN.z() * ray_length_;

        geometry_msgs::PointStamped pt_out;

        tf_listener_.transformPoint(laser_frame, pt, pt_out);

        tf::Vector3 transformedPt(pt_out.point.x, pt_out.point.y, pt_out.point.z);
        return transformedPt;
    };

    const ros::Time bbTime = bb_msg->header.stamp;

    std::vector<BoundingCylinder> bbCylinders;
    for (const auto& bb : bb_vec) {
        if (bb.probability < min_prob_ || classes_.count(bb.Class) != 1)
            continue;

        tf::Vector3 topLeft;
        tf::Vector3 bottomLeft;
        tf::Vector3 topRight;
        tf::Vector3 bottomRight;
        try {
            topLeft = projectPt(bb.xmin, bb.ymin);
            bottomLeft = projectPt(bb.xmin, bb.ymax);
            topRight = projectPt(bb.xmax, bb.ymin);
            bottomRight = projectPt(bb.xmax, bb.ymax);
        } catch (const tf::TransformException& ex) {
            ROS_WARN_STREAM("Failed to transform from camera to laser: " << ex.what());
            return;
        }

        // Laser angles are counter clockwise from x-axis
        const tf::Vector3 x_axis = tf::Vector3(1, 0, 0);
        float topLeftAngle = tf::Vector3(topLeft.x(), topLeft.y(), 0).angle(x_axis);
        float bottomLeftAngle = tf::Vector3(bottomLeft.x(), bottomLeft.y(), 0).angle(x_axis);
        float topRightAngle = tf::Vector3(topRight.x(), topRight.y(), 0).angle(x_axis);
        float bottomRightAngle = tf::Vector3(bottomRight.x(), bottomRight.y(), 0).angle(x_axis);

        tf::Vector3 left;
        float leftAngle;
        if (topLeftAngle < bottomLeftAngle) {
            left = {topLeft.x(), topLeft.y(), 0};
            leftAngle = topLeftAngle;
        } else {
            left = {bottomLeft.x(), bottomLeft.y(), 0};
            leftAngle = bottomLeftAngle;
        }
        left.normalize();

        tf::Vector3 right;
        float rightAngle;
        if (topRightAngle > bottomRightAngle) {
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

        bbCylinders.emplace_back(bb.Class, bb.id, bbTime, center, diameter / 2, height);
    }

    callback_(bbCylinders);
}

void BoundingBoxTransformer::handleLaserScan(
        const sensor_msgs::LaserScanConstPtr& laser_msg)
{
    if (last_lasers_.size() >= NUM_LASER_SCANS)
        last_lasers_.pop_back();

    last_lasers_.push_front(laser_msg);

    // FIXME: need a second transform when they aren't equal...
    if (laser_msg->header.frame_id != people_frame_) {
        ROS_ERROR_STREAM_THROTTLE(1.0, "Laser frame (" <<
                laser_msg->header.frame_id <<
                ") is different from people frame (" <<
                people_frame_ << ')');
    }
}

std::optional<sensor_msgs::LaserScanConstPtr> BoundingBoxTransformer::findLaserScanForTime(ros::Time t)
{
    double timeout = tf_timeout_.toSec();
    for (const auto& laser : last_lasers_) {
        if (std::abs((laser->header.stamp - t).toSec()) < timeout)
            return laser;
    }

    return {};
}

