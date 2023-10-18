#include "rio/gtsam/optimization.h"

#include <gtsam/base/timing.h>
#include <gtsam/geometry/BearingRange.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/slam/expressions.h>
#include <log++.h>
#include <tf2_eigen/tf2_eigen.h>

#include "rio/gtsam/doppler_factor.h"
#include "rio/gtsam/expressions.h"
using namespace rio;
using namespace gtsam;

typedef BearingRange<Pose3, Point3> BearingRange3D;

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::L;
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

void Optimization::addDopplerFactors(const Propagation& propagation,
                                     const gtsam::SharedNoiseModel& noise_model,
                                     std::vector<Vector1>* doppler_residuals) {
  if (!propagation.getLastStateIdx().has_value()) {
    LOG(E,
        "Propagation has no last state index, skipping adding Doppler "
        "factor.");
    return;
  }
  if (!propagation.cfar_detections_.has_value()) {
    LOG(I,
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
  auto state = propagation.getLatestState();
  Vector3 B_omega_IB;
  tf2::fromMsg(state->imu->angular_velocity, B_omega_IB);
  for (const auto& detection : propagation.cfar_detections_.value()) {
    // See https://dongjing3309.github.io/files/gtsam-tutorial.pdf
    // and adjoint-test.cpp for the derivation of the following.
    auto T_IB = Pose3_(X(idx));
    // Note(rikba): For calibration replace constant B_T_BR_ with symbol.
    auto T_BR = Pose3_(propagation.B_T_BR_.value());
    // TODO(rikba): Correct B_omega_IB bias.
    auto R_v_IR = unrotate(
        rotation(T_IB * T_BR),
        Vector3_(V(idx)) +
            rotate(rotation(T_IB), cross(B_omega_IB, translation(T_BR))));
    Point3 R_p_RT(detection.x, detection.y, detection.z);
    if (R_p_RT.norm() < 0.1) {
      LOG(E, "DopplerFactor: Radar point is too close to radar. Distance: "
                 << R_p_RT.norm() << "m");
      continue;
    }
    auto h = radialVelocity_(R_v_IR, Point3_(-R_p_RT));
    auto z = static_cast<double>(detection.velocity);
    new_graph_.addExpressionFactor(noise_model, z, h);

    // auto h = radialVelocity_(V(idx), L(detection.id));
    // Expression<BearingRange3D>(
    //     BearingRange3D::Measure,
    //     Pose3_(X(idx)) * Pose3_(propagation.B_T_BR_.value()),
    //     Point3_(L(track->getId())));
    // auto z = BearingRange3D(Pose3().bearing(R_p_RT), Pose3().range(R_p_RT));

    // new_graph_.addExpressionFactor(
    //     noise_model,
    //     DopplerFactor(X(idx), V(idx), B(idx),
    //                   {detection.x, detection.y, detection.z},
    //                   detection.velocity, I_omega_IB,
    //                   propagation.B_T_BR_.value(), noise_model),
    //     radialVelocity_(V(idx), L(detection.id))

    // auto factor = DopplerFactor(X(idx), V(idx), B(idx),
    //                             {detection.x, detection.y, detection.z},
    //                             detection.velocity, B_omega_IB,
    //                             propagation.B_T_BR_.value(), noise_model);
    // new_graph_.add(factor);
    // if (doppler_residuals) {
    //   doppler_residuals->emplace_back(
    //       factor.evaluateError(state->getPose(), state->I_v_IB,
    //                            state->getBias(), nullptr, nullptr, nullptr));
    // }
  }
}

template <>
void Optimization::addFactor<BearingRangeFactor<Pose3, Point3>>(
    const Propagation& propagation,
    const gtsam::SharedNoiseModel& noise_model) {
  if (!propagation.getLastStateIdx().has_value()) {
    LOG(E,
        "Propagation has no last state index, skipping adding bearing "
        "range factor.");
    return;
  }
  if (!propagation.cfar_tracks_.has_value()) {
    LOG(I,
        "Propagation has no CFAR tracks, skipping adding bearing "
        "range factor.");
    return;
  }
  if (!propagation.B_T_BR_.has_value()) {
    LOG(D,
        "Propagation has no B_t_BR, skipping adding bearing "
        "range factor.");
    return;
  }
  auto idx = propagation.getLastStateIdx().value();
  auto state = propagation.getLatestState();
  auto I_T_IR = propagation.getLatestState()->getPose().compose(
      propagation.B_T_BR_.value());
  for (const auto& track : propagation.cfar_tracks_.value()) {
    // Landmark in sensor frame.
    Point3 R_p_RT = track->getR_p_RT();
    auto h = Expression<BearingRange3D>(
        BearingRange3D::Measure,
        Pose3_(X(idx)) * Pose3_(propagation.B_T_BR_.value()),
        Point3_(L(track->getId())));
    auto z = BearingRange3D(Pose3().bearing(R_p_RT), Pose3().range(R_p_RT));
    new_graph_.addExpressionFactor(noise_model, z, h);
    new_timestamps_[L(track->getId())] = state->imu->header.stamp.toSec();
    if (!track->isAdded()) {
      auto I_p_IT = I_T_IR.transformFrom(R_p_RT);
      new_values_.insert(L(track->getId()), I_p_IT);
      track->setAdded();
      LOG(D, "Added landmark " + std::to_string(track->getId()) +
                     " at location I_T_IP: "
                 << I_T_IR.transformFrom(track->getR_p_RT()).transpose());
    }
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
    const gtsam::SharedNoiseModel& noise_model_radar_doppler,
    const gtsam::SharedNoiseModel& noise_model_radar_track,
    std::vector<Vector1>* doppler_residuals) {
  // TODO(rikba): Remove possible IMU factor between prev_state and next_state.

  // Add IMU factor from prev_state to split_state.
  addFactor<CombinedImuFactor>(propagation_to_radar);
  // Add IMU factor from split_state to next_state.
  addFactor<CombinedImuFactor>(propagation_from_radar);
  if (propagation_from_radar.getLastStateIdx().has_value()) {
    LOG(E, "Weird.");
  }

  // Add all doppler factors to split_state.
  addDopplerFactors(propagation_to_radar, noise_model_radar_doppler,
                    doppler_residuals);

  // Add all bearing range factors to split_state.
  addFactor<BearingRangeFactor<Pose3, Point3>>(propagation_to_radar,
                                               noise_model_radar_track);

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
  } else {
    LOG(E, "Propagation to radar has no last state index.");
  }
}

bool Optimization::solve(const std::deque<Propagation>& propagations) {
  if (running_.load()) {
    LOG(D, "Optimization thread still running.");
    return false;
  }
  if (thread_.joinable()) {
    LOG(D, "Optimization thread not joined, get result first.");
    return false;
  }

  running_.store(true);
  thread_ = std::thread(&Optimization::solveThreaded, this, new_graph_,
                        new_values_, new_timestamps_, propagations);

  new_graph_.resize(0);
  new_values_.clear();
  new_timestamps_.clear();
  return true;
}

bool Optimization::getResult(std::deque<Propagation>* propagation,
                             std::map<std::string, Timing>* timing) {
  if (running_.load()) {
    LOG(D, "Optimization thread still running.");
    return false;
  }
  if (thread_.joinable()) {
    thread_.join();
  } else {
    LOG(D, "Optimization thread is still running, skipping result.");
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!new_result_) {
    LOG(W, "No new result.");
    return false;
  }
  new_result_ = false;

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

  if (timing) *timing = timing_;
  return true;
}

void Optimization::solveThreaded(
    const gtsam::NonlinearFactorGraph graph, const gtsam::Values values,
    const gtsam::FixedLagSmoother::KeyTimestampMap stamps,
    std::deque<Propagation> propagations) {
  gttic_(optimize);
  try {
    smoother_.update(graph, values, stamps);
  } catch (const std::exception& e) {
    LOG(E, "Exception in update: " << e.what());
    running_.store(false);
    return;
  }
  gttoc_(optimize);

  // Update propagations.
  gttic_(cachePropagations);
  auto smallest_time = std::min_element(
      smoother_.timestamps().begin(), smoother_.timestamps().end(),
      [](const auto& a, const auto& b) { return a.second < b.second; });
  while (!propagations.empty() &&
         propagations.front().getFirstState()->imu->header.stamp.toSec() <
             smallest_time->second) {
    propagations.pop_front();
  }

  for (auto& propagation : propagations) {
    try {
      State initial_state(propagation.getFirstState()->odom_frame_id,
                          smoother_.calculateEstimate<gtsam::Pose3>(
                              X(propagation.getFirstStateIdx())),
                          smoother_.calculateEstimate<gtsam::Velocity3>(
                              V(propagation.getFirstStateIdx())),
                          propagation.getFirstState()->imu,
                          propagation.getFirstState()->integrator);
      initial_state.integrator.resetIntegrationAndSetBias(
          smoother_.calculateEstimate<gtsam::imuBias::ConstantBias>(
              B(propagation.getFirstStateIdx())));

      if (!propagation.repropagate(initial_state)) {
        LOG(E, "Failed to repropagate.");
        running_.store(false);
        return;
      }
    } catch (const std::exception& e) {
      LOG(E, "Exception in caching new values at idx: "
                 << propagation.getFirstStateIdx() << " Error: " << e.what());
      running_.store(false);
      return;
    }
  }
  gttoc_(cachePropagations);

  // Update member variables.
  std::lock_guard<std::mutex> lock(mutex_);
  propagations_ = propagations;

  tictoc_finishedIteration_();
  tictoc_getNode(optimize, optimize);
  updateTiming(optimize, "optimize",
               propagations.back().getLatestState()->imu->header.stamp);

  tictoc_getNode(cachePropagations, cachePropagations);
  updateTiming(cachePropagations, "cachePropagations",
               propagations.back().getLatestState()->imu->header.stamp);

  new_result_ = true;
  running_.store(false);
}

void Optimization::updateTiming(
    const std::shared_ptr<const ::gtsam::internal::TimingOutline>& variable,
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