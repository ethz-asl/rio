#include "rio/gtsam/propagation.h"

#include <algorithm>

#include <log++.h>
#include <tf2_eigen/tf2_eigen.h>

using namespace rio;
using namespace gtsam;

Propagation::Propagation(const State::ConstPtr& initial_state)
    : states_({initial_state}) {}

Propagation::Propagation(const std::vector<State::ConstPtr>& initial_states)
    : states_(initial_states) {
  LOG(I, "New propagation from " << states_.front()->imu->header.stamp << " to "
                                 << states_.back()->imu->header.stamp << ".");
}

bool Propagation::addImuMeasurement(const sensor_msgs::ImuConstPtr& msg) {
  if (states_.empty()) {
    LOG(E, "No initial state, skipping IMU integration.");
    return false;
  }
  if (states_.front() == nullptr) {
    LOG(E, "Initial state not complete, skipping IMU integration.");
    return false;
  }
  if (states_.back()->imu == nullptr) {
    LOG(E, "Previous IMU measurement not complete, skipping IMU integration.");
    return false;
  }

  auto dt = (msg->header.stamp - states_.back()->imu->header.stamp).toSec();
  if (dt < 0) {
    LOG(W, "Negative dt, skipping IMU integration.");
    return false;
  }
  Vector3 lin_acc, ang_vel;
  tf2::fromMsg(msg->linear_acceleration, lin_acc);
  tf2::fromMsg(msg->angular_velocity, ang_vel);
  auto integrator = states_.back()->integrator;
  integrator.integrateMeasurement(lin_acc, ang_vel, dt);
  auto prediction =
      integrator.predict(states_.front()->getNavState(), integrator.biasHat());
  states_.push_back(std::make_shared<State>(
      states_.back()->odom_frame_id, prediction.pose().translation(),
      prediction.pose().rotation(), prediction.velocity(), msg, integrator));
  return true;
}

bool Propagation::split(const ros::Time& t, Propagation* propagation_from_t) {
  if (states_.empty()) {
    LOG(W, "No initial state, skipping split.");
    return false;
  }
  if (states_.front() == nullptr) {
    LOG(W, "Initial state not complete, skipping split.");
    return false;
  }
  if (t < states_.front()->imu->header.stamp) {
    LOG(D, "t is before first IMU measurement, skipping split.");
    return false;
  }
  if (t > states_.back()->imu->header.stamp) {
    LOG(D, "t is after last IMU measurement, skipping split.");
    return false;
  }
  auto it =
      std::lower_bound(states_.begin(), states_.end(), t,
                       [](const State::ConstPtr& state, const ros::Time& t) {
                         return state->imu->header.stamp < t;
                       });
  if (it == states_.end()) {
    LOG(W, "Failed to find IMU measurement before t, skipping split.");
    return false;
  }
  auto idx = std::distance(states_.begin(), it);
  *propagation_from_t = Propagation(
      std::vector<State::ConstPtr>(states_.begin() + idx, states_.end()));
  states_.erase(states_.begin() + idx, states_.end());
  LOG(I, "Old propagation from " << states_.front()->imu->header.stamp
                                  << " to " << states_.back()->imu->header.stamp
                                  << ".");
  return true;
}