/*
 * This node loads a rosbag with IMU and radar data, sets up a calibration
 * optimization and solves for the states and extrinsic calibration.
 */

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/expressions.h>
#include <log++.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_eigen/tf2_eigen.h>

#include "rio/gtsam/expressions.h"
#include "rio/gtsam/optimization.h"
#include "rio/ros/common.h"

using namespace rio;
using namespace gtsam;
using namespace ros;

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::C;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

struct ImuMeasurement {
  double t;
  Vector3 a;
  Vector3 b_omega_ib;
};

struct RadarDetection {
  Vector3 R_p_RT;
  double v;
};

struct RadarMeasurement {
  double t;
  std::vector<RadarDetection> detections;
};

struct OdometryMeasurement {
  double t;
  Pose3 T_IB;
  Vector3 I_v_IB;
};

int main(int argc, char** argv) {
  init(argc, argv, "rio_calibration_node");
  NodeHandle nh;
  NodeHandle nh_private("~");

  // Load parameters.
  std::string bag_path;
  if (!loadParam<std::string>(nh_private, "bag_path", &bag_path)) {
    return 1;
  }
  std::string imu_topic;
  if (!loadParam<std::string>(nh_private, "imu_topic", &imu_topic)) {
    return 1;
  }
  std::string radar_topic;
  if (!loadParam<std::string>(nh_private, "radar_topic", &radar_topic)) {
    return 1;
  }
  std::string odometry_topic;
  if (!loadParam<std::string>(nh_private, "odometry_topic", &odometry_topic)) {
    return 1;
  }

  SharedNoiseModel radar_radial_velocity_noise_model;
  if (!loadNoiseRadarRadialVelocity(nh_private,
                                    &radar_radial_velocity_noise_model)) {
    return 1;
  }

  // Load rosbag.
  rosbag::Bag bag;
  bag.open(bag_path, rosbag::bagmode::Read);

  // Read IMU, IMU raw and radar data from rosbag
  std::vector<OdometryMeasurement> odometry_measurements;
  std::vector<ImuMeasurement> imu_raw_measurements;
  std::vector<RadarMeasurement> radar_measurements;

  rosbag::View view(bag);
  for (const rosbag::MessageInstance& msg : view) {
    if (msg.getTopic() == odometry_topic) {
      nav_msgs::OdometryConstPtr odom_msg =
          msg.instantiate<nav_msgs::Odometry>();
      Eigen::Quaterniond q_IB;
      tf2::fromMsg(odom_msg->pose.pose.orientation, q_IB);
      Vector3 I_t_IB;
      tf2::fromMsg(odom_msg->pose.pose.position, I_t_IB);
      OdometryMeasurement odometry_measurement;
      odometry_measurement.t = odom_msg->header.stamp.toSec();
      odometry_measurement.T_IB = {Rot3(q_IB), I_t_IB};
      Vector3 B_v_IB;
      tf2::fromMsg(odom_msg->twist.twist.linear, B_v_IB);
      odometry_measurement.I_v_IB = Rot3(q_IB).rotate(B_v_IB);
      odometry_measurements.push_back(odometry_measurement);
    } else if (msg.getTopic() == imu_topic) {
      sensor_msgs::ImuConstPtr imu_msg = msg.instantiate<sensor_msgs::Imu>();
      ImuMeasurement imu_measurement;
      imu_measurement.t = imu_msg->header.stamp.toSec();
      tf2::fromMsg(imu_msg->linear_acceleration, imu_measurement.a);
      tf2::fromMsg(imu_msg->angular_velocity, imu_measurement.b_omega_ib);
      imu_raw_measurements.push_back(imu_measurement);
    } else if (msg.getTopic() == radar_topic) {
      sensor_msgs::PointCloud2Ptr radar_msg =
          msg.instantiate<sensor_msgs::PointCloud2>();
      auto detections = parseRadarMsg(radar_msg);
      RadarMeasurement radar_measurement;
      radar_measurement.t = radar_msg->header.stamp.toSec();
      for (const auto& detection : detections) {
        RadarDetection radar_detection;
        radar_detection.R_p_RT = {detection.x, detection.y, detection.z};
        if (radar_detection.R_p_RT.norm() < 0.1) {
          LOG(W, "Ignoring radar measurement with detection distance "
                     << radar_detection.R_p_RT.norm());
          continue;
        }
        radar_detection.v = static_cast<double>(detection.velocity);
        radar_measurement.detections.push_back(radar_detection);
      }
      radar_measurements.push_back(radar_measurement);
    }
  }

  if (odometry_measurements.empty()) {
    LOG(F, "No odometry measurements found in rosbag.");
    return 1;
  }
  if (imu_raw_measurements.empty()) {
    LOG(F, "No IMU measurements found in rosbag.");
    return 1;
  }
  if (radar_measurements.empty()) {
    LOG(F, "No radar measurements found in rosbag.");
    return 1;
  }

  LOG(I, "Loaded " << odometry_measurements.size() << " odometry measurements, "
                   << imu_raw_measurements.size() << " IMU measurements and "
                   << radar_measurements.size() << " radar measurements.");

  // Find the time at which we have all measurements.
  double t_start =
      std::max({odometry_measurements.front().t, imu_raw_measurements.front().t,
                radar_measurements.front().t});
  // Remove measurements before t_start.
  odometry_measurements.erase(
      odometry_measurements.begin(),
      std::lower_bound(
          odometry_measurements.begin(), odometry_measurements.end(), t_start,
          [](const OdometryMeasurement& m, double t) { return m.t < t; }));
  imu_raw_measurements.erase(
      imu_raw_measurements.begin(),
      std::lower_bound(
          imu_raw_measurements.begin(), imu_raw_measurements.end(), t_start,
          [](const ImuMeasurement& m, double t) { return m.t < t; }));
  radar_measurements.erase(
      radar_measurements.begin(),
      std::lower_bound(
          radar_measurements.begin(), radar_measurements.end(), t_start,
          [](const RadarMeasurement& m, double t) { return m.t < t; }));

  // Create nonlinear factor graph.
  NonlinearFactorGraph graph;
  Values values;
  std::map<size_t, double> idx_stamp_map;

  // Add a radar factor for each radar measurement.
  size_t idx = 0;
  for (const auto& radar_measurement : radar_measurements) {
    idx_stamp_map[idx] = radar_measurement.t;
    // Initial value close to current index.
    // Could find actual closest time, but this is good enough.
    auto odom = std::lower_bound(
        odometry_measurements.begin(), odometry_measurements.end(),
        radar_measurement.t,
        [](const OdometryMeasurement& m, double t) { return m.t < t; });
    if (odom == odometry_measurements.end()) continue;
    values.insert(X(idx), odom->T_IB);
    values.insert(V(idx), odom->I_v_IB);
    values.insert(B(idx), imuBias::ConstantBias());
    values.insert(C(idx), Pose3());

    // Radar factor.
    auto T_IB = Pose3_(X(idx));
    auto T_BR = Pose3_(C(idx));
    // Find the next IMU message to determine angular velocity.
    auto imu = std::lower_bound(
        imu_raw_measurements.begin(), imu_raw_measurements.end(),
        radar_measurement.t,
        [](const ImuMeasurement& m, double t) { return m.t < t; });
    auto R_v_IR = unrotate(
        rotation(T_IB * T_BR),
        Vector3_(V(idx)) + rotate(rotation(T_IB),
                                  cross(correctGyroscope_(ConstantBias_(B(idx)),
                                                          imu->b_omega_ib),
                                        translation(T_BR))));
    idx++;

    for (const auto& detection : radar_measurement.detections) {
      auto h = radialVelocity_(R_v_IR, Point3_(-detection.R_p_RT));
      auto z = detection.v;
      auto factor = ExpressionFactor(radar_radial_velocity_noise_model, z, h);
      graph.add(factor);
    }
  }

  return 0;
}