/*
BSD 3-Clause License

Copyright (c) 2024 ETH Zurich, Autonomous Systems Lab, Rik Girod

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "rio/gtsam/state.h"

#include <log++.h>
#include <tf2_eigen/tf2_eigen.h>

using namespace rio;
using namespace gtsam;

State::State(const std::string& odom_frame_id, const gtsam::Point3& I_p_IB,
             const gtsam::Rot3& R_IB, const gtsam::Vector3& I_v_IB,
             const sensor_msgs::ImuConstPtr& imu,
             const gtsam::PreintegratedCombinedMeasurements& integrator,
             const std::optional<double>& baro_height_bias)
    : odom_frame_id(odom_frame_id),
      I_p_IB(I_p_IB),
      R_IB(R_IB),
      I_v_IB(I_v_IB),
      imu(imu),
      integrator(integrator),
      baro_height_bias(baro_height_bias) {}

State::State(const std::string& odom_frame_id, const gtsam::Pose3& I_T_IB,
             const gtsam::Vector3& I_v_IB, const sensor_msgs::ImuConstPtr& imu,
             const gtsam::PreintegratedCombinedMeasurements& integrator,
             const std::optional<double>& baro_height_bias)
    : State(odom_frame_id, I_T_IB.translation(), I_T_IB.rotation(), I_v_IB, imu,
            integrator, baro_height_bias) {}

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