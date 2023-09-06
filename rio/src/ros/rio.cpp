#include "rio/ros/rio.h"

#include <log++.h>
#include <nav_msgs/Odometry.h>
#include <tf2_eigen/tf2_eigen.h>

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
      nh_.subscribe("imu/data_raw", queue_size, &Rio::imuRawCallback, this);
  imu_filter_sub_ =
      nh_.subscribe("imu/data", queue_size, &Rio::imuFilterCallback, this);
  radar_trigger_sub_ = nh_.subscribe("radar/trigger", queue_size,
                                     &Rio::radarTriggerCallback, this);
  radar_cfar_sub_ = nh_.subscribe("radar/cfar_detections", queue_size,
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
  Eigen::Quaterniond q_IB;
  tf2::fromMsg(msg->orientation, q_IB);
  initial_state_.q_IB = Rot3(q_IB);
}

void Rio::radarTriggerCallback(const std_msgs::HeaderConstPtr& msg) {
  LOG_FIRST(I, 1, "Received first radar trigger message.");
}

void Rio::cfarDetectionsCallback(const sensor_msgs::PointCloud2& msg) {
  LOG_FIRST(I, 1, "Received first CFAR detections.");
}

bool Rio::State::isComplete() const {
  return I_p_IB.has_value() && q_IB.has_value() && I_v_IB.has_value() &&
         b_a.has_value() && b_g.has_value();
}

bool Rio::State::reset() {
  I_p_IB.reset();
  q_IB.reset();
  I_v_IB.reset();
  b_a.reset();
  b_g.reset();
  return true;
}