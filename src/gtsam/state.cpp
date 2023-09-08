#include "rio/gtsam/state.h"

#include <log++.h>
#include <tf2_eigen/tf2_eigen.h>

using namespace rio;
using namespace gtsam;

State::State(const std::string& odom_frame_id, const gtsam::Point3& I_p_IB,
             const gtsam::Rot3& R_IB, const gtsam::Vector3& I_v_IB,
             const sensor_msgs::ImuConstPtr& imu,
             const gtsam::PreintegratedCombinedMeasurements& integrator)
    : odom_frame_id_(odom_frame_id),
      I_p_IB_(I_p_IB),
      R_IB_(R_IB),
      I_v_IB_(I_v_IB),
      imu_(imu),
      integrator_(integrator) {}

nav_msgs::Odometry State::getOdometry() const {
  nav_msgs::Odometry odom;
  odom.header.stamp = imu_->header.stamp;
  odom.header.frame_id = odom_frame_id_;
  odom.child_frame_id = imu_->header.frame_id;
  odom.pose.pose.orientation = tf2::toMsg(R_IB_.toQuaternion());
  odom.pose.pose.position = tf2::toMsg(I_p_IB_);
  tf2::toMsg(R_IB_.unrotate(I_v_IB_), odom.twist.twist.linear);
  odom.twist.twist.angular = imu_->angular_velocity;
  return odom;
}

geometry_msgs::TransformStamped State::getTransform() const {
  geometry_msgs::TransformStamped transform;
  transform.header.stamp = imu_->header.stamp;
  transform.header.frame_id = odom_frame_id_;
  transform.child_frame_id = imu_->header.frame_id;
  transform.transform.rotation = tf2::toMsg(R_IB_.toQuaternion());
  tf2::toMsg(I_p_IB_, transform.transform.translation);
  return transform;
}

geometry_msgs::Vector3Stamped State::getBiasAcc() const {
  geometry_msgs::Vector3Stamped bias_acc;
  bias_acc.header.stamp = imu_->header.stamp;
  bias_acc.header.frame_id = imu_->header.frame_id;
  tf2::toMsg(integrator_.biasHat().accelerometer(), bias_acc.vector);
  return bias_acc;
}

geometry_msgs::Vector3Stamped State::getBiasGyro() const {
  geometry_msgs::Vector3Stamped bias_gyro;
  bias_gyro.header.stamp = imu_->header.stamp;
  bias_gyro.header.frame_id = imu_->header.frame_id;
  tf2::toMsg(integrator_.biasHat().gyroscope(), bias_gyro.vector);
  return bias_gyro;
}

NavState State::getNavState() const {
  return NavState(R_IB_, I_p_IB_, I_v_IB_);
}