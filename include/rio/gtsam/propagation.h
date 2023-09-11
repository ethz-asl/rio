#pragma once

#include <vector>

#include "rio/gtsam/state.h"
#include "sensor_msgs/Imu.h"

namespace rio {

class Propagation {
 public:
  inline Propagation(){};
  Propagation(const State& initial_state);
  Propagation(const State::ConstPtr& initial_state);
  Propagation(const std::vector<State::ConstPtr>& initial_states);
  bool addImuMeasurement(const sensor_msgs::ImuConstPtr& msg);
  bool addImuMeasurement(const sensor_msgs::Imu& msg);
  inline State::ConstPtr getLatestState() const { return states_.back(); }
  bool split(const ros::Time& t, Propagation* propagation_to_t,
             Propagation* propagation_from_t) const;

 private:
  // Vector of IMU measurements and preintegration up to this IMU.
  std::vector<State::ConstPtr> states_;
};

}  // namespace rio