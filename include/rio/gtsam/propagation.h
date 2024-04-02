#pragma once

#include <optional>
#include <vector>

#include <mav_sensors_drivers/sensor_types/Radar.h>

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
  std::optional<std::vector<mav_sensors::Radar::CfarDetection>>
      cfar_detections_;
  std::optional<std::vector<Track::Ptr>> cfar_tracks_;
  std::optional<double> baro_height_;

 private:
  // Vector of IMU measurements and preintegration up to this IMU.
  std::vector<State::ConstPtr> states_;

  uint64_t first_state_idx_{0};
  std::optional<uint64_t> last_state_idx_;
};

}  // namespace rio