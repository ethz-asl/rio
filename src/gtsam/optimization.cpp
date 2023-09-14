#include "rio/gtsam/optimization.h"

#include <gtsam/base/timing.h>
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

bool Optimization::solve(const std::deque<Propagation>& propagations) {
  if (thread_.joinable()) {
    LOG(D, "Optimization thread not joined, get result first.");
    return false;
  }

  // Create deep copy.
  auto graph = new_graph_.clone();
  auto values = new_values_;
  auto stamps = new_timestamps_;
  new_graph_.resize(0);
  new_values_.clear();
  new_timestamps_.clear();

  propagations_ = propagations;

  thread_ =
      std::thread(&Optimization::solveThreaded, this, graph, values, stamps);
  return true;
}

bool Optimization::getResult(std::deque<Propagation>* propagation,
                             std::map<std::string, Timing>* timing) {
  if (thread_.joinable()) {
    thread_.join();
  } else {
    LOG(D, "Optimization thread is still running, skipping result.");
    return false;
  }
  if (!new_result_) {
    LOG(W, "No new result.");
    return false;
  }

  // Pop all propagations previous to the current propagation result, i.e.,
  // states that have been marginalized out.
  gttic_(deqeueCleanup);
  while (!propagation->empty() &&
         propagation->front().getFirstStateIdx() !=
             propagations_.front().getFirstStateIdx()) {
    propagation->pop_front();
  }
  gttoc_(deqeueCleanup);

  // Replace all propagations that have been updated with the new result.
  gttic_(copyCachedPropagations);
  std::set<std::deque<Propagation>::iterator> updated;
  for (auto it = propagation->begin(); it != propagation->end(); ++it) {
    auto result_it = std::find_if(
        propagations_.begin(), propagations_.end(), [it](const auto& p) {
          return p.getFirstStateIdx() == it->getFirstStateIdx() &&
                 p.getLastStateIdx().has_value() &&
                 it->getLastStateIdx().has_value() &&
                 p.getLastStateIdx().value() == it->getLastStateIdx().value();
        });
    if (result_it != propagations_.end()) {
      *it = *result_it;
      updated.insert(it);
      if (result_it == propagations_.begin())
        propagations_.pop_front();  // Cleanup.
    }
  }
  gttoc_(copyCachedPropagations);

  // Repropagate all remaining propagations.
  gttic_(repropagateNewPropagations);
  for (auto it = propagation->begin(); it != propagation->end(); ++it) {
    if (updated.count(it) > 0) continue;
    if (it == propagation->begin()) {
      LOG(E, "First propagation not updated, skipping.");
      continue;
    }
    if (!it->repropagate(*(std::prev(it)->getLatestState()))) {
      LOG(E, "Failed to repropagate.");
      continue;
    }
  }
  gttoc_(repropagateNewPropagations);

  tictoc_finishedIteration_();
  tictoc_getNode(deqeueCleanup, deqeueCleanup);
  updateTiming(deqeueCleanup, "deqeueCleanup",
               timing_["optimize"].header.stamp);
  tictoc_getNode(copyCachedPropagations, copyCachedPropagations);
  updateTiming(copyCachedPropagations, "copyCachedPropagations",
               timing_["optimize"].header.stamp);
  tictoc_getNode(repropagateNewPropagations, repropagateNewPropagations);
  updateTiming(repropagateNewPropagations, "repropagateNewPropagations",
               timing_["optimize"].header.stamp);

  *timing = timing_;
  return true;
}

void Optimization::solveThreaded(
    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
    const gtsam::FixedLagSmoother::KeyTimestampMap& stamps) {
  gttic_(optimize);
  try {
    smoother_.update(graph, values, stamps);
  } catch (const std::exception& e) {
    LOG(E, "Exception in update: " << e.what());
    return;
  }

  Values new_values;
  try {
    new_values = smoother_.calculateEstimate();
  } catch (const std::exception& e) {
    LOG(E, "Exception in calculateEstimate: " << e.what());
    return;
  }
  gttoc_(optimize);

  // Update propagations.
  gttic_(cachePropagations);
  auto smallest_time = std::min_element(
      smoother_.timestamps().begin(), smoother_.timestamps().end(),
      [](const auto& a, const auto& b) { return a.second < b.second; });
  while (!propagations_.empty() &&
         propagations_.front().getFirstState()->imu->header.stamp.toSec() <
             smallest_time->second) {
    propagations_.pop_front();
  }

  for (auto& propagation : propagations_) {
    try {
      State initial_state(
          propagation.getFirstState()->odom_frame_id,
          new_values.at<gtsam::Pose3>(X(propagation.getFirstStateIdx())),
          new_values.at<gtsam::Vector3>(V(propagation.getFirstStateIdx())),
          propagation.getFirstState()->imu,
          propagation.getFirstState()->integrator);
      initial_state.integrator.resetIntegrationAndSetBias(
          new_values.at<gtsam::imuBias::ConstantBias>(
              B(propagation.getFirstStateIdx())));

      if (!propagation.repropagate(initial_state)) {
        LOG(E, "Failed to repropagate.");
        return;
      }
    } catch (const std::exception& e) {
      LOG(E, "Exception in getting new values at idx: "
                 << propagation.getFirstStateIdx() << " Error: " << e.what());
      return;
    }
  }
  gttoc_(cachePropagations);

  tictoc_finishedIteration_();
  tictoc_getNode(optimize, optimize);
  updateTiming(optimize, "optimize",
               propagations_.back().getLatestState()->imu->header.stamp);

  tictoc_getNode(cachePropagations, cachePropagations);
  updateTiming(cachePropagations, "cachePropagations",
               propagations_.back().getLatestState()->imu->header.stamp);

  new_result_ = true;
}

void Optimization::updateTiming(
    const boost::shared_ptr<const gtsam::internal::TimingOutline>& variable,
    const std::string& label, const ros::Time& stamp) {
  if (timing_.find(label) == timing_.end()) {
    timing_[label] = Timing();
  }
  timing_[label].header.stamp = stamp;
  timing_[label].header.frame_id = label;
  timing_[label].iteration = variable->self() - timing_[label].total;
  timing_[label].total = variable->self();
  timing_[label].min = variable->min();
  timing_[label].max = variable->max();
  timing_[label].mean = variable->mean();
}