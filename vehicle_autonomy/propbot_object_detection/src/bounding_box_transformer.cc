#include <bounding_box_transformer.hh>

#include <bounding_cylinder.hh>

#include <darknet_ros_msgs/BoundingBox.h>

#include <vector>
#include <algorithm>
#include <numeric>
#include <utility>
#include <limits>
#include <cassert>

std::pair<float, float> BoundingBoxTransformer::findLaserDepthMinMax(
        float angle_min, float angle_max)
{
    // Narrow the bounding box a little to try to remove extreme values
    // this is a crappy hack and should be fixed with proper filtering
    angle_min += 0.04;
    angle_max -= 0.04;

    if (angle_max < angle_min)
        angle_max = angle_min;

    std::vector<float> found_ranges;

    float sum = 0.0;
    float min = std::numeric_limits<float>::max();
    float max = 0.0;
    size_t count = 0;
    for (int i = 0; i < last_laser_->ranges.size(); i++) {
        float range = last_laser_->ranges[i];
        if (range < last_laser_->range_min || last_laser_->range_max < range)
            continue;

        float angle = last_laser_->angle_min + i * last_laser_->angle_increment;
        if (angle_min <= angle && angle <= angle_max) {
            sum += range;
            count++;

            if (range < min)
                min = range;

            if (range > max)
                max = range;
        }
    }

    if (count > 0) {
        float average = sum / count;
        // TODO: some kind of heuristics to filter our extreme values
        //
        // currently fixed to 1m in diameter since we are detecting people
        return {average - 0.5, average + 0.5};
    } else {
        return {-1.0, -1.0};
    }
}

void BoundingBoxTransformer::handleCameraImage(
        const sensor_msgs::ImageConstPtr& image_msg,
        const sensor_msgs::CameraInfoConstPtr& cam_info_msg)
{
    ROS_INFO_STREAM_THROTTLE(1, "Received Camera Image");
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

    if (!last_laser_) {
        ROS_WARN_STREAM_THROTTLE(1, "Waiting for Laser Scan");
        return;
    }

    tf::StampedTransform camera_to_laser;
    if (!getCameraTfTo(last_laser_->header.frame_id, camera_to_laser))
        return;

    auto& bb_vec = bb_msg->bounding_boxes;

    // Find a 3D point (in the laser frame) corresponding to the pixel
    // this is done by extending the ray from camera to pixel out to ray length
    auto projectPt = [&](int x, int y) -> tf::Vector3 {
        auto cvRay = pinhole_camera_.projectPixelTo3dRay(cv::Point2d(x, y));

        // Rays originate from the camera origin, no offset needed
        tf::Vector3 ray(cvRay.x, cvRay.y, cvRay.z);
        ray.normalize();
        ray *= ray_length_;

        return camera_to_laser * ray;
    };

    const ros::Time bbTime = bb_msg->header.stamp;

    std::vector<BoundingCylinder> bbCylinders;
    for (const auto& bb : bb_vec) {
        if (bb.probability < min_prob_ || classes_.count(bb.Class) != 1)
            continue;

        tf::Vector3 topLeft = projectPt(bb.xmin, bb.ymin);
        tf::Vector3 bottomLeft = projectPt(bb.xmin, bb.ymax);
        tf::Vector3 topRight = projectPt(bb.xmax, bb.ymin);
        tf::Vector3 bottomRight = projectPt(bb.xmax, bb.ymax);

        // Laser angles are counter clockwise from x-axis
        const tf::Vector3 x_axis = tf::Vector3(1, 0, 0);
        float topLeftAngle = topLeft.angle(x_axis);
        float bottomLeftAngle = bottomLeft.angle(x_axis);
        float topRightAngle = topRight.angle(x_axis);
        float bottomRightAngle = bottomRight.angle(x_axis);

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

        assert(leftAngle < rightAngle);

        auto [min, max] = findLaserDepthMinMax(leftAngle, rightAngle);
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
    last_laser_ = laser_msg;

    // FIXME: need a second transform when they aren't equal...
    if (last_laser_->header.frame_id != people_frame_) {
        ROS_ERROR_STREAM_THROTTLE(1.0, "Laser frame (" <<
                last_laser_->header.frame_id <<
                ") is different from people frame (" <<
                people_frame_ << ')');
    }
}

bool BoundingBoxTransformer::getCameraTfTo(
        const std::string& frame, tf::StampedTransform transform)
{
    try {
        // Find the transform
        tf_listener_.waitForTransform(frame,
                pinhole_camera_.tfFrame(),
                pinhole_camera_.stamp(),
                tf_timeout_);
        tf_listener_.lookupTransform(frame,
                pinhole_camera_.tfFrame(),
                pinhole_camera_.stamp(),
                transform);
        return true;
    } catch (const tf::TransformException& ex) {
        ROS_WARN_STREAM("Failed to get transform from camera to " << frame <<
                ": " << ex.what());
    }

    return false;
}

