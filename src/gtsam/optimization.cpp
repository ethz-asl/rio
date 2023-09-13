#include "rio/gtsam/optimization.h"

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <log++.h>
#include <tf2_eigen/tf2_eigen.h>

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
void Optimization::addFactor<CombinedImuFactor>(
    const Propagation& propagation,
    const gtsam::SharedNoiseModel& noise_model) {
  auto first_idx = propagation.getFirstStateIdx();
  auto first_state = propagation.getFirstState();
  auto second_idx = propagation.getLastStateIdx();
  auto second_state = propagation.getLatestState();
  if (!second_idx.has_value()) {
    LOG(D, "Propagation has no last state index, skipping adding IMU factor.");
    return;
  }
  new_graph_.add(CombinedImuFactor(
      X(first_idx), V(first_idx), X(second_idx.value()), V(second_idx.value()),
      B(first_idx), B(second_idx.value()), second_state->integrator));
}

template <>
void Optimization::addFactor<DopplerFactor>(
    const Propagation& propagation,
    const gtsam::SharedNoiseModel& noise_model) {
  if (!propagation.getLastStateIdx().has_value()) {
    LOG(D,
        "Propagation has no last state index, skipping adding Doppler "
        "factor.");
    return;
  }
  if (!propagation.cfar_detections_.has_value()) {
    LOG(D,
        "Propagation has no CFAR detections, skipping adding Doppler factor.");
    return;
  }
  if (!propagation.B_T_BR_.has_value()) {
    LOG(D,
        "Propagation has no B_t_BR, skipping adding Doppler "
        "factor.");
    return;
  }
  auto idx = propagation.getLastStateIdx().value();
  Vector3 I_omega_IB;
  tf2::fromMsg(propagation.getLatestState()->imu->angular_velocity, I_omega_IB);
  for (const auto& detection : propagation.cfar_detections_.value()) {
    new_graph_.add(DopplerFactor(X(idx), V(idx), B(idx),
                                 {detection.x, detection.y, detection.z},
                                 detection.velocity, I_omega_IB,
                                 propagation.B_T_BR_.value(), noise_model));
  }
}

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
  // TODO(rikba): Remove possible IMU factor between prev_state and next_state.

  // Add IMU factor from prev_state to split_state.
  addFactor<CombinedImuFactor>(propagation_to_radar);
  // Add IMU factor from split_state to next_state.
  addFactor<CombinedImuFactor>(propagation_from_radar);

  // Add all radar factors to split_state.
  addFactor<DopplerFactor>(propagation_to_radar, noise_model_radar);

  // Add initial state at split_state.
  if (propagation_to_radar.getLastStateIdx().has_value()) {
    auto idx = propagation_to_radar.getLastStateIdx().value();
    auto state = propagation_to_radar.getLatestState();
    new_values_.insert(X(idx), state->getPose());
    new_timestamps_[X(idx)] = state->imu->header.stamp.toSec();
    new_values_.insert(V(idx), state->I_v_IB);
    new_timestamps_[V(idx)] = state->imu->header.stamp.toSec();
    new_values_.insert(B(idx), state->getBias());
    new_timestamps_[B(idx)] = state->imu->header.stamp.toSec();
  }
}

bool Optimization::solve() {
  if (!thread_.joinable()) {
    LOG(D, "Optimization thread is not joinable, skipping optimization.");
    return false;
  } else {
    thread_.join();
  }

  // Create deep copy.
  auto graph = std::make_unique<gtsam::NonlinearFactorGraph>(new_graph_);
  auto values = std::make_unique<gtsam::Values>(new_values_);
  auto stamps = std::make_unique<gtsam::FixedLagSmoother::KeyTimestampMap>(
      new_timestamps_);
  new_graph_.resize(0);
  new_values_.clear();
  new_timestamps_.clear();

  thread_ = std::thread(&Optimization::solveThreaded, this, std::move(graph),
                        std::move(values), std::move(stamps));
  return true;
}

void Optimization::solveThreaded(
    std::unique_ptr<gtsam::NonlinearFactorGraph> graph,
    std::unique_ptr<gtsam::Values> values,
    std::unique_ptr<gtsam::FixedLagSmoother::KeyTimestampMap> stamps) {
      
    }