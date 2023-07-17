#pragma once

#include <ros/ros.h>

namespace rio
{
    ros::Time toRosTime(uint64_t unix_stamp_ns)
    {
        // Convert nanoseconds to seconds and remaining nanoseconds.
        uint64_t sec = unix_stamp_ns * 1e-9;
        uint64_t nsec = unix_stamp_ns - sec * 1e9;
        return ros::Time(sec, nsec);
    }
}