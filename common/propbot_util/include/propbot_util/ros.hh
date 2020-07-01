#pragma once

#include <ros/ros.h>

#include <string>

namespace propbot {
/**
 * Helper to try to get a parameter, using a default if it fails
 */
template <typename T>
inline T getParamOrDefault(ros::NodeHandle nh, const std::string& key, T def)
{
  T val;
  if (nh.getParam(key, val))
    return val;
  else
    return def;
}
}
