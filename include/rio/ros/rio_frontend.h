#pragma once

#include <optional>

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>

#include "rio/gtsam/state.h"

// This class implements a callback driven sensor fusion.
// IMU raw callback: preintegrate IMU measurements and publish the result.
// IMU filter callback: get initial orientation from external filter.
// radar trigger callback: create new key.
// radar measurement callback: add radar measurement to the current key, start
// optimization thread.
namespace rio {
class RioFrontend {
 public:
  RioFrontend(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  bool init();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber imu_raw_sub_;
  ros::Subscriber imu_filter_sub_;
  ros::Subscriber radar_trigger_sub_;
  ros::Subscriber radar_cfar_sub_;
  void imuRawCallback(const sensor_msgs::ImuConstPtr& msg);
  void imuFilterCallback(const sensor_msgs::ImuConstPtr& msg);
  void radarTriggerCallback(const std_msgs::HeaderConstPtr& msg);
  void cfarDetectionsCallback(const sensor_msgs::PointCloud2& msg);

  ros::Publisher odom_navigation_pub_;
  ros::Publisher odom_optimizer_pub_;

  // Set unknown initial states to zero.
  State initial_state_{.I_p_IB = {gtsam::Z_3x1}, .I_v_IB = {gtsam::Z_3x1}};
  State navigation_state_{};
  std::optional<State::Vector> states_;

  // IMU preintegration.
  gtsam::PreintegratedCombinedMeasurements integrator_;
};
}  // namespace rio