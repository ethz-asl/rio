#pragma once

#include <memory>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

namespace rio {

struct State {
  typedef std::shared_ptr<State> Ptr;
  typedef std::shared_ptr<const State> ConstPtr;

  State() = delete;
  State(const std::string& odom_frame_id, const gtsam::Point3& I_p_IB,
        const gtsam::Rot3& R_IB, const gtsam::Vector3& I_v_IB,
        const sensor_msgs::ImuConstPtr& imu,
        const gtsam::PreintegratedCombinedMeasurements& integrator);

  bool operator==(const State& other) const;
  inline bool operator!=(const State& other) const { return !(*this == other); }

  void print(const std::string& s = "") const;

  std::string odom_frame_id;
  gtsam::Point3 I_p_IB;
  gtsam::Rot3 R_IB;
  gtsam::Vector3 I_v_IB;
  sensor_msgs::ImuConstPtr imu;
  gtsam::PreintegratedCombinedMeasurements integrator;

  nav_msgs::Odometry getOdometry() const;
  geometry_msgs::TransformStamped getTransform() const;
  geometry_msgs::Vector3Stamped getBiasAcc() const;
  geometry_msgs::Vector3Stamped getBiasGyro() const;
  gtsam::NavState getNavState() const;
  gtsam::imuBias::ConstantBias getBias() const;
};
}  // namespace rio