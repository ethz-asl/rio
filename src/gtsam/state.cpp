#include "rio/gtsam/state.h"

#include <log++.h>
#include <tf2_eigen/tf2_eigen.h>

using namespace rio;
using namespace gtsam;

State::State(const std::string& odom_frame_id, const gtsam::Point3& I_p_IB,
             const gtsam::Rot3& R_IB, const gtsam::Vector3& I_v_IB,
             const sensor_msgs::ImuConstPtr& imu,
             const gtsam::PreintegratedCombinedMeasurements& integrator)
    : odom_frame_id(odom_frame_id),
      I_p_IB(I_p_IB),
      R_IB(R_IB),
      I_v_IB(I_v_IB),
      imu(imu),
      integrator(integrator) {}

nav_msgs::Odometry State::getOdometry() const {
  nav_msgs::Odometry odom;
  odom.header.stamp = imu->header.stamp;
  odom.header.frame_id = odom_frame_id;
  odom.child_frame_id = imu->header.frame_id;
  odom.pose.pose.orientation = tf2::toMsg(R_IB.toQuaternion());
  odom.pose.pose.position = tf2::toMsg(I_p_IB);
  tf2::toMsg(R_IB.unrotate(I_v_IB), odom.twist.twist.linear);
  odom.twist.twist.angular = imu->angular_velocity;
  return odom;
}

geometry_msgs::TransformStamped State::getTransform() const {
  geometry_msgs::TransformStamped transform;
  transform.header.stamp = imu->header.stamp;
  transform.header.frame_id = odom_frame_id;
  transform.child_frame_id = imu->header.frame_id;
  transform.transform.rotation = tf2::toMsg(R_IB.toQuaternion());
  tf2::toMsg(I_p_IB, transform.transform.translation);
  return transform;
}

geometry_msgs::Vector3Stamped State::getBiasAcc() const {
  geometry_msgs::Vector3Stamped bias_acc;
  bias_acc.header.stamp = imu->header.stamp;
  bias_acc.header.frame_id = imu->header.frame_id;
  tf2::toMsg(integrator.biasHat().accelerometer(), bias_acc.vector);
  return bias_acc;
}

geometry_msgs::Vector3Stamped State::getBiasGyro() const {
  geometry_msgs::Vector3Stamped bias_gyro;
  bias_gyro.header.stamp = imu->header.stamp;
  bias_gyro.header.frame_id = imu->header.frame_id;
  tf2::toMsg(integrator.biasHat().gyroscope(), bias_gyro.vector);
  return bias_gyro;
}

gtsam::imuBias::ConstantBias State::getBias() const {
  return integrator.biasHat();
}

NavState State::getNavState() const { return NavState(R_IB, I_p_IB, I_v_IB); }

Pose3 State::getPose() const { return Pose3(R_IB, I_p_IB); }

bool State::operator==(const State& other) const {
  return odom_frame_id == other.odom_frame_id &&
         gtsam::equal_with_abs_tol(I_p_IB, other.I_p_IB) &&
         R_IB.equals(other.R_IB) &&
         gtsam::equal_with_abs_tol(I_v_IB, other.I_v_IB) && imu == other.imu &&
         integrator.equals(other.integrator);
}

void State::print(const std::string& s) const {
  LOG(I, s.c_str() << "  state:");
  LOG(I, s.c_str() << "    odom_frame_id: " << odom_frame_id);
  LOG(I, s.c_str() << "    I_p_IB: " << I_p_IB.transpose());
  LOG(I, s.c_str() << "    R_IB:\n" << R_IB);
  LOG(I, s.c_str() << "    I_v_IB: " << I_v_IB.transpose());
  LOG(I, s.c_str() << "    imu: " << *imu);
  integrator.print();
}