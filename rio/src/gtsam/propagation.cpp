#include "rio/gtsam/propagation.h"

#include <algorithm>

#include <log++.h>
#include <tf2_eigen/tf2_eigen.h>

using namespace rio;
using namespace gtsam;

Propagation::Propagation(const State& initial_state,
                         const uint64_t first_state_idx,
                         const std::optional<uint64_t>& last_state_idx)
    : Propagation(std::make_shared<State>(initial_state), first_state_idx,
                  last_state_idx) {}

Propagation::Propagation(const State::ConstPtr& initial_state,
                         const uint64_t first_state_idx,
                         const std::optional<uint64_t>& last_state_idx)
    : Propagation(std::vector<State::ConstPtr>({initial_state}),
                  first_state_idx, last_state_idx) {}

Propagation::Propagation(const std::vector<State::ConstPtr>& initial_states,
                         const uint64_t first_state_idx,
                         const std::optional<uint64_t>& last_state_idx)
    : states_(initial_states),
      first_state_idx_(first_state_idx),
      last_state_idx_(last_state_idx) {}

bool Propagation::addImuMeasurement(const sensor_msgs::Imu& msg) {
  return addImuMeasurement(sensor_msgs::ImuConstPtr(new sensor_msgs::Imu(msg)));
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
  if (dt == 0) {
    LOG(W, "Zero dt, skipping IMU integration.");
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

bool Propagation::split(const ros::Time& t, uint64_t* split_idx,
                        Propagation* propagation_to_t,
                        Propagation* propagation_from_t) const {
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
  auto state_1 =
      std::lower_bound(states_.begin(), states_.end(), t,
                       [](const State::ConstPtr& state, const ros::Time& t) {
                         return state->imu->header.stamp < t;
                       });
  if (state_1 == states_.begin()) {
    LOG(W, "Failed to find IMU measurement after t, skipping split.");
    return false;
  }
  if (state_1 == states_.end()) {
    LOG(W, "Failed to find IMU measurement before t, skipping split.");
    return false;
  }
  auto state_0 = std::prev(state_1);

  // Create ZOH IMU message to propagate to t.
  sensor_msgs::Imu imu;
  imu.header.stamp = t;
  imu.header.frame_id = (*state_1)->imu->header.frame_id;
  imu.linear_acceleration = (*state_1)->imu->linear_acceleration;
  imu.angular_velocity = (*state_1)->imu->angular_velocity;

  *propagation_to_t =
      Propagation(std::vector<State::ConstPtr>(states_.begin(), state_1),
                  first_state_idx_, (*split_idx));
  if (t > (*state_0)->imu->header.stamp)
    propagation_to_t->addImuMeasurement(imu);
  else
    LOG(W, "Split before or exactly at measurement time. t_split: "
               << t << " t_0: " << (*state_0)->imu->header.stamp);

  // Regenerate propagation from t.
  if (t < (*state_1)->imu->header.stamp) {
    State initial_state = {propagation_to_t->getLatestState()->odom_frame_id,
                           propagation_to_t->getLatestState()->I_p_IB,
                           propagation_to_t->getLatestState()->R_IB,
                           propagation_to_t->getLatestState()->I_v_IB,
                           propagation_to_t->getLatestState()->imu,
                           propagation_to_t->getLatestState()->integrator};
    initial_state.integrator.resetIntegrationAndSetBias(
        propagation_to_t->getLatestState()->integrator.biasHat());
    *propagation_from_t =
        Propagation(initial_state, (*split_idx), last_state_idx_);
    for (auto it = state_1; it != states_.end(); ++it) {
      propagation_from_t->addImuMeasurement((*it)->imu);
    }
  } else {
    *propagation_from_t =
        Propagation(std::vector<State::ConstPtr>(state_1, states_.end()),
                    (*split_idx), last_state_idx_);
    LOG(W, "Split after or exactly at measurement time. t_split: "
               << t << " t_1: " << (*state_1)->imu->header.stamp);
  }
  (*split_idx)++;

  return true;
}