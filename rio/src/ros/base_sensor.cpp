#include "rio/ros/base_sensor.h"

#include <log++.h>
#include <std_msgs/Header.h>

using namespace rio;

BaseSensor::BaseSensor(const ros::NodeHandle& nh_private) : nh_private_(nh_private) {}

bool BaseSensor::init() {
  // Open sensor.
  if (!openSensor()) {
    LOG(F, "Failed to open sensor.");
    return false;
  }

  // Advertise timestamp.
  trigger_pub_ = nh_private_.advertise<std_msgs::Header>("trigger", 1);

  // Get sensor frame_id.
  if (!nh_private_.getParam("frame_id", frame_id_)) {
    LOG(F, "Failed to read frame_id.");
    return false;
  }

  // Initialize timer.
  double rate;
  if (!nh_private_.getParam("rate", rate)) {
    LOG(F, "Failed to read rate.");
    return false;
  }
  timer_ = nh_private_.createTimer(ros::Duration(1.0 / rate), &BaseSensor::timerCallback, this);

  return true;
}

void BaseSensor::timerCallback(const ros::TimerEvent& event) {
  // Publish timestamp.
  std_msgs::Header msg;
  msg.stamp = ros::Time::now();
  msg.frame_id = frame_id_;
  trigger_pub_.publish(msg);

  // Read sensor data.
  readSensor();
}