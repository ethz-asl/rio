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

#include "rio/gtsam/propagation.h"

#include <algorithm>

#include <log++.h>
#include <tf2_eigen/tf2_eigen.h>

using namespace rio;
using namespace gtsam;

Propagation::Propagation(const State& state,
                          const uint64_t idx)
                          : state_(state),
                            graph_idx_(idx) {}

bool Propagation::addImuMeasurement(const sensor_msgs::Imu& msg) {
  return addImuMeasurement(sensor_msgs::ImuConstPtr(new sensor_msgs::Imu(msg)));
}

bool Propagation::addImuMeasurement(const sensor_msgs::ImuConstPtr& msg) {
  auto dt = (msg->header.stamp - state_.imu->header.stamp).toSec();
  if (dt < 0) {
    LOG(W, "Negative dt, skipping IMU integration.");
    return false;
  }
  if (dt == 0) {
    LOG(W, "Zero dt, skipping IMU integration.");
    return false;
  }
  Vector3 lin_acc, ang_vel;
  tf2::fromMsg(msg->linear_acceleration, lin_acc);
  tf2::fromMsg(msg->angular_velocity, ang_vel);
  state_.integrator.integrateMeasurement(lin_acc, ang_vel, dt);
  auto prediction =
      state_.integrator.predict(state_.getNavState(), state_.integrator.biasHat());
  state_ = State(state_.odom_frame_id, prediction.pose().translation(),
                 prediction.pose().rotation(), prediction.velocity(), msg,
                 state_.integrator);
  imu_measurements_.push_back(msg);
  return true;
}

bool Propagation::split(const ros::Time& t, uint64_t* split_idx,
                        Propagation* propagation_to_t,
                        Propagation* propagation_from_t) {
  auto partition_iter =
      std::partition(imu_measurements_.begin(), imu_measurements_.end(),
                     [&t](const sensor_msgs::ImuConstPtr& imu) {
                       return imu->header.stamp <= t;
                     });
  propagation_to_t->imu_measurements_ = std::vector<sensor_msgs::ImuConstPtr>(
      imu_measurements_.begin(), partition_iter);
  if (propagation_to_t->imu_measurements_.empty()) {
    LOG(W, "No IMU measurements in propagation to t, skipping split.");
    return false;
  }
  propagation_from_t->imu_measurements_ = std::vector<sensor_msgs::ImuConstPtr>(
      partition_iter, imu_measurements_.end());
  propagation_to_t->repropagate();

  if (t > propagation_to_t->imu_measurements_.back()->header.stamp) {

    sensor_msgs::Imu imu;
    imu.header = propagation_to_t->imu_measurements_.back()->header;
    imu.header.stamp = t;
    imu.linear_acceleration = propagation_to_t->imu_measurements_.back()->linear_acceleration;
    imu.angular_velocity = propagation_to_t->imu_measurements_.back()->angular_velocity;
    propagation_to_t->addImuMeasurement(imu);
  }
  propagation_to_t->graph_idx_ = *split_idx;
  propagation_from_t->state_ = propagation_to_t->state_;

  propagation_from_t->repropagate();
  return true;
}