#include "rio/gtsam/optimization.h"

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/PriorFactor.h>

#include "rio/gtsam/doppler_factor.h"

using namespace rio;
using namespace gtsam;

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

template <>
void Optimization::addFactor<PriorFactor<Pose3>>(
    const uint32_t idx, const State::ConstPtr& state,
    const gtsam::SharedNoiseModel& noise_model) {
  new_values_.insert(X(idx), state->getPose());
  new_timestamps_[X(idx)] = state->imu->header.stamp.toSec();
  new_graph_.add(PriorFactor<Pose3>(X(idx), state->getPose(), noise_model));
}

template <>
void Optimization::addFactor<PriorFactor<Vector3>>(
    const uint32_t idx, const State::ConstPtr& state,
    const gtsam::SharedNoiseModel& noise_model) {
  new_values_.insert(V(idx), state->I_v_IB);
  new_timestamps_[V(idx)] = state->imu->header.stamp.toSec();
  new_graph_.add(PriorFactor<Vector3>(V(idx), state->I_v_IB, noise_model));
}

template <>
void Optimization::addFactor<PriorFactor<imuBias::ConstantBias>>(
    const uint32_t idx, const State::ConstPtr& state,
    const gtsam::SharedNoiseModel& noise_model) {}

template <>
void Optimization::addFactor<DopplerFactor>(
    const uint32_t idx, const State::ConstPtr& state,
    const gtsam::SharedNoiseModel& noise_model) {}

void Optimization::addPriorFactor(
    const State::ConstPtr& state,
    const gtsam::SharedNoiseModel& noise_model_I_T_IB,
    const gtsam::SharedNoiseModel& noise_model_I_v_IB,
    const gtsam::SharedNoiseModel& noise_model_imu_bias) {
  addFactor<PriorFactor<Pose3>>(idx_, state, noise_model_I_T_IB);
  addFactor<PriorFactor<Vector3>>(idx_, state, noise_model_I_v_IB);
  addFactor<PriorFactor<imuBias::ConstantBias>>(idx_, state,
                                                noise_model_imu_bias);
  ++idx_;
}

void Optimization::addRadarFactor(
    const State::ConstPtr& prev_state, const State::ConstPtr& split_state,
    const State::ConstPtr& next_state,
    const gtsam::SharedNoiseModel& noise_model_radar) {
        // Check if IMU factor between prev_state and next_state exists.
        // If yes, remove it. (TODO)
        
        // Add IMU factor from prev_state to split_state.

        // Add all radar factors to split_state.
    }