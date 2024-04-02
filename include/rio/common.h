#pragma once

#include <ros/ros.h>

namespace rio {
ros::Time toRosTime(uint64_t unix_stamp_ns);
}