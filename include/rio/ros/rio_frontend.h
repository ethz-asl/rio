#pragma once

#include <deque>
#include <vector>

#include <gtsam/linear/NoiseModel.h>
#include <mav_sensors_drivers/sensor_types/Radar.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>

#include "rio/gtsam/optimization.h"
#include "rio/gtsam/propagation.h"
#include "rio/gtsam/state.h"

// This class implements a callback driven sensor fusion.
// IMU raw callback: preintegrate IMU measurements and publish the result.
// IMU filter callback: get initial orientation from external filter.
// radar measurement callback: split preintegration to add radar factor, start
// optimization.
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
  void cfarDetectionsCallback(const sensor_msgs::PointCloud2Ptr& msg);

  ros::Publisher odom_navigation_pub_;
  ros::Publisher odom_optimizer_pub_;

  State::ConstPtr initial_state_{std::make_shared<State>(
      "odom", gtsam::Z_3x1, gtsam::Rot3(), gtsam::Z_3x1, nullptr,
      gtsam::PreintegratedCombinedMeasurements())};
  std::deque<Propagation> propagation_;
  ros::Duration max_dead_reckoning_duration_{60.0};

  std::deque<Propagation>::iterator splitPropagation(const ros::Time& t);
  void popOldPropagations();

  Optimization optimization_;
  gtsam::noiseModel::Diagonal::shared_ptr prior_noise_model_I_T_IB_;
  gtsam::noiseModel::Diagonal::shared_ptr prior_noise_model_I_v_IB_;
  gtsam::noiseModel::Diagonal::shared_ptr prior_noise_model_imu_bias_;
  gtsam::noiseModel::Diagonal::shared_ptr noise_model_radar_;
  uint64_t idx_{0};

  std::vector<mav_sensors::Radar::CfarDetection> parseRadarMsg(
      const sensor_msgs::PointCloud2Ptr& msg) const;
};
}  // namespace rio