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
    const Propagation& propagation,
    const gtsam::SharedNoiseModel& noise_model) {
  auto idx = propagation.getFirstStateIdx();
  auto state = propagation.getFirstState();
  new_values_.insert(X(idx), state->getPose());
  new_timestamps_[X(idx)] = state->imu->header.stamp.toSec();
  new_graph_.add(PriorFactor<Pose3>(X(idx), state->getPose(), noise_model));
}

template <>
void Optimization::addFactor<PriorFactor<Vector3>>(
    const Propagation& propagation,
    const gtsam::SharedNoiseModel& noise_model) {
  auto idx = propagation.getFirstStateIdx();
  auto state = propagation.getFirstState();
  new_values_.insert(V(idx), state->I_v_IB);
  new_timestamps_[V(idx)] = state->imu->header.stamp.toSec();
  new_graph_.add(PriorFactor<Vector3>(V(idx), state->I_v_IB, noise_model));
}

template <>
void Optimization::addFactor<PriorFactor<imuBias::ConstantBias>>(
    const Propagation& propagation,
    const gtsam::SharedNoiseModel& noise_model) {
  auto idx = propagation.getFirstStateIdx();
  auto state = propagation.getFirstState();
  new_values_.insert(B(idx), state->getBias());
  new_timestamps_[B(idx)] = state->imu->header.stamp.toSec();
  new_graph_.add(PriorFactor<imuBias::ConstantBias>(B(idx), state->getBias(),
                                                    noise_model));
}

template <>
void Optimization::addFactor<DopplerFactor>(
    const Propagation& propagation,
    const gtsam::SharedNoiseModel& noise_model) {}

void Optimization::addPriorFactor(
    const Propagation& propagation,
    const gtsam::SharedNoiseModel& noise_model_I_T_IB,
    const gtsam::SharedNoiseModel& noise_model_I_v_IB,
    const gtsam::SharedNoiseModel& noise_model_imu_bias) {
  addFactor<PriorFactor<Pose3>>(propagation, noise_model_I_T_IB);
  addFactor<PriorFactor<Vector3>>(propagation, noise_model_I_v_IB);
  addFactor<PriorFactor<imuBias::ConstantBias>>(propagation,
                                                noise_model_imu_bias);
}

void Optimization::addRadarFactor(
    const Propagation& propagation_to_radar,
    const Propagation& propagation_from_radar,
    const gtsam::SharedNoiseModel& noise_model_radar) {
  // Remove possible IMU factor between prev_state and next_state.


  // Add IMU factor from prev_state to split_state.

  // Add all radar factors to split_state.
}