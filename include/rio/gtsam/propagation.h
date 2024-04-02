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

#include <optional>
#include <vector>

#include "rio/common.h"
#include "rio/gtsam/landmark_tracker.h"
#include "rio/gtsam/state.h"
#include "sensor_msgs/Imu.h"

namespace rio {

class Propagation {
 public:
  inline Propagation(){};
  Propagation(const State& initial_state, const uint64_t first_state_idx,
              const std::optional<uint64_t>& last_state_idx = std::nullopt);
  Propagation(const State::ConstPtr& initial_state,
              const uint64_t first_state_idx,
              const std::optional<uint64_t>& last_state_idx = std::nullopt);
  Propagation(const std::vector<State::ConstPtr>& initial_states,
              const uint64_t first_state_idx,
              const std::optional<uint64_t>& last_state_idx = std::nullopt);
  bool addImuMeasurement(const sensor_msgs::ImuConstPtr& msg);
  bool addImuMeasurement(const sensor_msgs::Imu& msg);
  inline State::ConstPtr getLatestState() const { return states_.back(); }
  inline State::ConstPtr getFirstState() const { return states_.front(); }
  bool split(const ros::Time& t, uint64_t* split_idx,
             Propagation* propagation_to_t,
             Propagation* propagation_from_t) const;
  bool repropagate(const State& initial_state);

  inline uint64_t getFirstStateIdx() const { return first_state_idx_; }
  inline std::optional<uint64_t> getLastStateIdx() const {
    return last_state_idx_;
  }

  std::optional<gtsam::Pose3> B_T_BR_;
  std::optional<std::vector<CfarDetection>> cfar_detections_;
  std::optional<std::vector<Track::Ptr>> cfar_tracks_;
  std::optional<double> baro_height_;

 private:
  // Vector of IMU measurements and preintegration up to this IMU.
  std::vector<State::ConstPtr> states_;

  uint64_t first_state_idx_{0};
  std::optional<uint64_t> last_state_idx_;
};

}  // namespace rio