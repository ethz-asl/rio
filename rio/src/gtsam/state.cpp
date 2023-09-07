#include "rio/gtsam/state.h"

#include <log++.h>
#include <tf2_eigen/tf2_eigen.h>

using namespace rio;
using namespace gtsam;

bool State::hasFullState() const {
  return I_p_IB.has_value() && R_IB.has_value() && I_v_IB.has_value() &&
         b_a.has_value() && b_g.has_value();
}

nav_msgs::Odometry State::getOdometry() const {
  nav_msgs::Odometry odom;
  if (stamp.has_value())
    odom.header.stamp = stamp.value();
  else
    LOG(W, "getOdometry: stamp not set");
  if (odom_frame_id.has_value())
    odom.header.frame_id = odom_frame_id.value();
  else
    LOG(W, "getOdometry: odom_frame_id not set");
  if (body_frame_id.has_value())
    odom.child_frame_id = body_frame_id.value();
  else
    LOG(W, "getOdometry: body_frame_id not set");
  if (R_IB.has_value())
    odom.pose.pose.orientation = tf2::toMsg(R_IB.value().toQuaternion());
  else
    LOG(W, "getOdometry: R_IB not set");
  if (I_p_IB.has_value())
    odom.pose.pose.position = tf2::toMsg(I_p_IB.value());
  else
    LOG(W, "getOdometry: I_p_IB not set");
  if (I_v_IB.has_value() && R_IB.has_value())
    tf2::toMsg(R_IB.value().unrotate(I_v_IB.value()), odom.twist.twist.linear);
  else
    LOG(W, "getOdometry: I_v_IB not set");
  if (imu.has_value())
    odom.twist.twist.angular = imu.value()->angular_velocity;
  else
    LOG(W, "getOdometry: IMU not set");

  return odom;
}

geometry_msgs::TransformStamped State::getTransform() const {
  geometry_msgs::TransformStamped transform;
  if (stamp.has_value())
    transform.header.stamp = stamp.value();
  else
    LOG(W, "getTransform: stamp not set");
  if (odom_frame_id.has_value())
    transform.header.frame_id = odom_frame_id.value();
  else
    LOG(W, "getTransform: odom_frame_id not set");
  if (body_frame_id.has_value())
    transform.child_frame_id = body_frame_id.value();
  else
    LOG(W, "getTransform: body_frame_id not set");
  if (R_IB.has_value())
    transform.transform.rotation = tf2::toMsg(R_IB.value().toQuaternion());
  else
    LOG(W, "getTransform: R_IB not set");
  if (I_p_IB.has_value())
    tf2::toMsg(I_p_IB.value(), transform.transform.translation);
  else
    LOG(W, "getTransform: I_p_IB not set");
  return transform;
}

geometry_msgs::Vector3Stamped State::getBiasAcc() const {
  geometry_msgs::Vector3Stamped bias_acc;
  if (stamp.has_value())
    bias_acc.header.stamp = stamp.value();
  else
    LOG(W, "getBiasAcc: stamp not set");
  if (body_frame_id.has_value())
    bias_acc.header.frame_id = body_frame_id.value();
  else
    LOG(W, "getBiasAcc: body_frame_id not set");
  if (b_a.has_value())
    tf2::toMsg(b_a.value(), bias_acc.vector);
  else
    LOG(W, "getBiasAcc: b_a not set");
  return bias_acc;
}

geometry_msgs::Vector3Stamped State::getBiasGyro() const {
  geometry_msgs::Vector3Stamped bias_gyro;
  if (stamp.has_value())
    bias_gyro.header.stamp = stamp.value();
  else
    LOG(W, "getBiasGyro: stamp not set");
  if (body_frame_id.has_value())
    bias_gyro.header.frame_id = body_frame_id.value();
  else
    LOG(W, "getBiasGyro: body_frame_id not set");
  if (b_g.has_value())
    tf2::toMsg(b_g.value(), bias_gyro.vector);
  else
    LOG(W, "getBiasGyro: b_g not set");
  return bias_gyro;
}

NavState State::getNavState() const {
  if (R_IB.has_value() && I_p_IB.has_value() && I_v_IB.has_value())
    return NavState(R_IB.value(), I_p_IB.value(), I_v_IB.value());
  else {
    LOG(W, "State not complete, returning empty NavState.");
    return NavState();
  }
}

imuBias::ConstantBias State::getBias() const {
  if (b_a.has_value() && b_g.has_value())
    return imuBias::ConstantBias(b_a.value(), b_g.value());
  else {
    LOG(W, "State not complete, returning empty bias.");
    return imuBias::ConstantBias();
  }
}
