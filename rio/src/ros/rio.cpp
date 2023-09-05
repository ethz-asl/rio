#include "rio/ros/rio.h"

#include <log++.h>
#include <nav_msgs/Odometry.h>

using namespace rio;
using namespace gtsam;

Rio::Rio(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {}

Rio::~Rio() {}

bool Rio::init() {
  int32_t queue_size = 1;
  if (!nh_private_.getParam("queue_size", queue_size)) {
    LOG(F, "Failed to read queue_size.");
    return false;
  }

  imu_raw_sub_ =
      nh_.subscribe("imu_raw", queue_size, &Rio::imuRawCallback, this);
  imu_filter_sub_ =
      nh_.subscribe("imu_filter", queue_size, &Rio::imuFilterCallback, this);
  radar_trigger_sub_ = nh_.subscribe("radar_trigger", queue_size,
                                     &Rio::radarTriggerCallback, this);
  radar_cfar_sub_ = nh_.subscribe("cfar_detections", queue_size,
                                  &Rio::cfarDetectionsCallback, this);

  odom_integrated_pub_ = nh_private_.advertise<nav_msgs::Odometry>(
      "odometry_integrated", queue_size);
  odom_optimized_pub_ = nh_private_.advertise<nav_msgs::Odometry>(
      "odometry_optimized", queue_size);
  return true;
}

void Rio::imuRawCallback(const sensor_msgs::ImuConstPtr& msg) {
  LOG_FIRST(I, 1, "Received first raw IMU message.");
}

void Rio::imuFilterCallback(const sensor_msgs::ImuConstPtr& msg) {
  LOG_FIRST(I, 1, "Received first filtered IMU message.");
}

void Rio::radarTriggerCallback(const std_msgs::HeaderConstPtr& msg) {
  LOG_FIRST(I, 1, "Received first radar trigger message.");
}

void Rio::cfarDetectionsCallback(const sensor_msgs::PointCloud2& msg) {
  LOG_FIRST(I, 1, "Received first CFAR detections.");
}
