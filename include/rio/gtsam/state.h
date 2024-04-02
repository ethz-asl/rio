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

#pragma once

#include <memory>
#include <optional>

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
        const gtsam::PreintegratedCombinedMeasurements& integrator,
        const std::optional<double>& baro_height_bias = std::nullopt);
  State(const std::string& odom_frame_id, const gtsam::Pose3& I_T_IB,
        const gtsam::Vector3& I_v_IB, const sensor_msgs::ImuConstPtr& imu,
        const gtsam::PreintegratedCombinedMeasurements& integrator,
        const std::optional<double>& baro_height_bias = std::nullopt);

  bool operator==(const State& other) const;
  inline bool operator!=(const State& other) const { return !(*this == other); }

  void print(const std::string& s = "") const;

  std::string odom_frame_id;
  gtsam::Point3 I_p_IB;
  gtsam::Rot3 R_IB;
  gtsam::Vector3 I_v_IB;
  sensor_msgs::ImuConstPtr imu;
  gtsam::PreintegratedCombinedMeasurements integrator;

  std::optional<double> baro_height_bias;

  nav_msgs::Odometry getOdometry() const;
  geometry_msgs::TransformStamped getTransform() const;
  geometry_msgs::Vector3Stamped getBiasAcc() const;
  geometry_msgs::Vector3Stamped getBiasGyro() const;
  gtsam::NavState getNavState() const;
  gtsam::Pose3 getPose() const;
  gtsam::imuBias::ConstantBias getBias() const;
};
}  // namespace rio