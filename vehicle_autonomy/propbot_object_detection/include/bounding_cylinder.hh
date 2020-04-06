#pragma once

#include <ros/ros.h>

#include <tf/transform_datatypes.h>

#include <string>

/**
 * A bounding box which is a cylinder instead of a box.
 *
 * The cylinder is always oriented vertically with respect to the ground.
 */
struct BoundingCylinder
{
  std::string objectClass;
  int id;
  ros::Time creationTime;
  tf::Point center;
  tfScalar radius;
  tfScalar height;

  BoundingCylinder(std::string objectClass, int id, ros::Time creationTime,
      tf::Point center, tfScalar radius, tfScalar height)
    : objectClass(objectClass)
    , id(id)
    , creationTime(creationTime)
    , center(center)
    , radius(radius)
    , height(height)
  {}
};

