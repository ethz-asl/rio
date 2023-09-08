#pragma once

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
  State() = delete;
  State(const std::string& odom_frame_id, const gtsam::Point3& I_p_IB,
        const gtsam::Rot3& R_IB, const gtsam::Vector3& I_v_IB,
        const sensor_msgs::ImuConstPtr& imu,
        const gtsam::PreintegratedCombinedMeasurements& integrator);

  inline std::string getOdomFrameId() const { return odom_frame_id_; }
  inline gtsam::Point3 getI_p_IB() const { return I_p_IB_; }
  inline gtsam::Rot3 getR_IB() const { return R_IB_; }
  inline gtsam::Vector3 getI_v_IB() const { return I_v_IB_; }
  inline sensor_msgs::ImuConstPtr getImu() const { return imu_; }
  inline gtsam::PreintegratedCombinedMeasurements getIntegrator() const {
    return integrator_;
  }

  nav_msgs::Odometry getOdometry() const;
  geometry_msgs::TransformStamped getTransform() const;
  geometry_msgs::Vector3Stamped getBiasAcc() const;
  geometry_msgs::Vector3Stamped getBiasGyro() const;
  gtsam::NavState getNavState() const;
  gtsam::imuBias::ConstantBias getBias() const;

 private:
  std::string odom_frame_id_;
  gtsam::Point3 I_p_IB_;
  gtsam::Rot3 R_IB_;
  gtsam::Vector3 I_v_IB_;
  sensor_msgs::ImuConstPtr imu_;
  gtsam::PreintegratedCombinedMeasurements integrator_;
};
}  // namespace rio