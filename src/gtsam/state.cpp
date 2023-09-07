#include "rio/gtsam/state.h"

#include <log++.h>
#include <tf2_eigen/tf2_eigen.h>

using namespace rio;
using namespace gtsam;

bool State::isComplete() const {
  return stamp.has_value() && odom_frame_id.has_value() &&
         body_frame_id.has_value() && I_p_IB.has_value() && R_IB.has_value() &&
         I_v_IB.has_value() && b_a.has_value() && b_g.has_value();
}

bool State::reset() {
  I_p_IB.reset();
  R_IB.reset();
  I_v_IB.reset();
  b_a.reset();
  b_g.reset();
  return true;
}

nav_msgs::Odometry State::getOdometry() const {
  nav_msgs::Odometry odom;
  if (!isComplete()) {
    LOG(W, "State not complete, returning empty odometry.");
    return odom;
  }
  odom.header.stamp = stamp.value();
  odom.header.frame_id = odom_frame_id.value();
  odom.child_frame_id = body_frame_id.value();
  odom.pose.pose.orientation = tf2::toMsg(R_IB.value().toQuaternion());
  odom.pose.pose.position = tf2::toMsg(I_p_IB.value());
  tf2::toMsg(R_IB.value().unrotate(I_v_IB.value()), odom.twist.twist.linear);

  return odom;
}

geometry_msgs::TransformStamped State::getTransform() const {
  geometry_msgs::TransformStamped transform;
  if (!isComplete()) {
    LOG(W, "State not complete, returning empty transform.");
    return transform;
  }

  transform.header.stamp = stamp.value();
  transform.header.frame_id = odom_frame_id.value();
  transform.child_frame_id = body_frame_id.value();
  transform.transform.rotation = tf2::toMsg(R_IB.value().toQuaternion());
  tf2::toMsg(I_p_IB.value(), transform.transform.translation);
  return transform;
}

geometry_msgs::Vector3Stamped State::getBiasAcc() const {
  geometry_msgs::Vector3Stamped bias_acc;
  if (!isComplete()) {
    LOG(W, "State not complete, returning empty bias acc.");
    return bias_acc;
  }
  bias_acc.header.stamp = stamp.value();
  bias_acc.header.frame_id = body_frame_id.value();
  tf2::toMsg(b_a.value(), bias_acc.vector);
  return bias_acc;
}

geometry_msgs::Vector3Stamped State::getBiasGyro() const {
  geometry_msgs::Vector3Stamped bias_gyro;
  if (!isComplete()) {
    LOG(W, "State not complete, returning empty bias gyro.");
    return bias_gyro;
  }
  bias_gyro.header.stamp = stamp.value();
  bias_gyro.header.frame_id = body_frame_id.value();
  tf2::toMsg(b_g.value(), bias_gyro.vector);
  return bias_gyro;
}

NavState State::getNavState() const {
  if (!isComplete()) {
    LOG(W, "State not complete, returning empty NavState.");
    return NavState();
  }
  return NavState(R_IB.value(), I_p_IB.value(), I_v_IB.value());
}

imuBias::ConstantBias State::getBias() const {
  if (!isComplete()) {
    LOG(W, "State not complete, returning empty bias.");
    return imuBias::ConstantBias();
  }
  return imuBias::ConstantBias(b_a.value(), b_g.value());
}
