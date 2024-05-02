#include "rio/gtsam/optimization.h"

#include <gtsam/base/timing.h>
#include <gtsam/geometry/BearingRange.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/slam/expressions.h>
#include <log++.h>
#include <tf2_eigen/tf2_eigen.h>

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
  auto idx = propagation.graph_idx_;
  auto state = propagation.state_;
  new_values_.insert(X(idx), state.getPose());
  new_timestamps_[X(idx)] = state.imu->header.stamp.toSec();
  new_graph_.add(PriorFactor<Pose3>(X(idx), state.getPose(), noise_model));
}

template <>
void Optimization::addFactor<PriorFactor<Vector3>>(
    const Propagation& propagation,
    const gtsam::SharedNoiseModel& noise_model) {
  auto idx = propagation.graph_idx_;
  auto state = propagation.state_;
  new_values_.insert(V(idx), state.I_v_IB);
  new_timestamps_[V(idx)] = state.imu->header.stamp.toSec();
  new_graph_.add(PriorFactor<Vector3>(V(idx), state.I_v_IB, noise_model));
}

template <>
void Optimization::addFactor<PriorFactor<imuBias::ConstantBias>>(
    const Propagation& propagation,
    const gtsam::SharedNoiseModel& noise_model) {
  auto idx = propagation.graph_idx_;
  auto state = propagation.state_;
  new_values_.insert(B(idx), state.getBias());
  new_timestamps_[B(idx)] = state.imu->header.stamp.toSec();
  new_graph_.add(
      PriorFactor<imuBias::ConstantBias>(B(idx), state.getBias(), noise_model));
}

template <>
void Optimization::addFactor<CombinedImuFactor>(
    const Propagation& propagation,
    const gtsam::SharedNoiseModel& noise_model) {
  auto first_idx = propagation.prior->graph_idx_;
  auto second_idx = propagation.graph_idx_;
  auto second_state = propagation.state_;

  new_graph_.add(CombinedImuFactor(X(first_idx), V(first_idx), X(second_idx),
                                   V(second_idx), B(first_idx), B(second_idx),
                                   second_state.integrator));
}

void Optimization::addDopplerFactors(const Propagation& propagation,
                                     const gtsam::SharedNoiseModel& noise_model,
                                     std::vector<Vector1>* doppler_residuals) {
  auto idx = propagation.graph_idx_;
  auto state = propagation.state_;
  Vector3 B_omega_IB;
  tf2::fromMsg(state.imu->angular_velocity, B_omega_IB);
  double doppler_sum = 0;
  for (const auto& detection : propagation.cfar_detections_.value()) {
    // See https://dongjing3309.github.io/files/gtsam-tutorial.pdf
    auto T_IB = Pose3_(X(idx));
    auto T_BR = Pose3_(propagation.B_T_BR_.value());
    // R_v_IR = R_RI * (I_v_IB + R_IB * (B_omega_IB x B_t_BR))
    auto R_v_IR = unrotate(
        rotation(T_IB * T_BR),
        Vector3_(V(idx)) +
            rotate(rotation(T_IB),
                   cross(correctGyroscope_(ConstantBias_(B(idx)), B_omega_IB),
                         translation(T_BR))));
    Point3 R_p_RT(detection.x, detection.y, detection.z);
    if (R_p_RT.norm() < 0.1) {
      LOG(E,
          "Radial velocity factor: Radar point is too close to radar. "
          "Distance: "
              << R_p_RT.norm() << "m");
      continue;
    }
    Unit3_ R_p_TR_unit(Unit3(-R_p_RT));
    auto h = radialVelocity_(R_v_IR, R_p_TR_unit);
    auto z = static_cast<double>(detection.velocity);
    auto factor = ExpressionFactor(noise_model, z, h);
    new_graph_.add(factor);
    if (doppler_residuals) {
      Values x;
      x.insert(X(idx), state.getPose());
      x.insert(V(idx), state.I_v_IB);
      x.insert(B(idx), state.getBias());
      doppler_residuals->emplace_back(factor.unwhitenedError(x));
    }
    doppler_sum += abs(static_cast<double>(detection.velocity));
  }
  
  if (doppler_sum == 0 && state.I_v_IB.norm() < 0.07) {
    // TODO: put this into a yaml file
    auto nm = noiseModel::Diagonal::Sigmas(Vector3(0.05, 0.05, 0.05));
    new_graph_.add(PriorFactor<Vector3>(V(idx), Vector3(0., 0., 0.), nm));
  } else if (doppler_sum == 0) {
    LOG(W, "zero doppler sum, but velocity is not zero: " << state.I_v_IB.norm()
                                                          << " m/s");
  } 
}

template <>
void Optimization::addFactor<BearingRangeFactor<Pose3, Point3>>(
    const Propagation& propagation,
    const gtsam::SharedNoiseModel& noise_model) {
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
  auto idx = propagation.graph_idx_;
  auto state = propagation.state_;
  auto I_T_IR = state.getPose().compose(propagation.B_T_BR_.value());
  for (const auto& track : propagation.cfar_tracks_.value()) {
    // Landmark in sensor frame.
    Point3 R_p_RT = track->getR_p_RT();
    auto h = Expression<BearingRange3D>(
        BearingRange3D::Measure,
        Pose3_(X(idx)) * Pose3_(propagation.B_T_BR_.value()),
        Point3_(L(track->getId())));
    auto z = BearingRange3D(Pose3().bearing(R_p_RT), Pose3().range(R_p_RT));
    new_graph_.addExpressionFactor(noise_model, z, h);
    new_timestamps_[L(track->getId())] = state.imu->header.stamp.toSec();
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

void Optimization::addPriorFactor(
    const Propagation& propagation, const State& initial_state,
    const gtsam::SharedNoiseModel& noise_model_I_T_IB,
    const gtsam::SharedNoiseModel& noise_model_I_v_IB) {
  auto idx = propagation.graph_idx_;
  auto state = propagation.state_;
  new_graph_.add(
      PriorFactor<Pose3>(X(idx), initial_state.getPose(), noise_model_I_T_IB));
  new_graph_.add(
      PriorFactor<Vector3>(V(idx), initial_state.I_v_IB, noise_model_I_v_IB));
}

void Optimization::addRadarFactor(
    const Propagation& propagation_to_radar,
    const Propagation& propagation_from_radar,
    const gtsam::SharedNoiseModel& noise_model_radar_doppler,
    const gtsam::SharedNoiseModel& noise_model_radar_track,
    std::vector<Vector1>* doppler_residuals) {
  gttic_(addRadarFactor);
  // TODO(rikba): Remove possible IMU factor between prev_state and next_state.

  // Add IMU factor from prev_state to split_state.
  addFactor<CombinedImuFactor>(propagation_to_radar);
  // Add IMU factor from split_state to next_state.
  if (propagation_from_radar.cfar_detections_.has_value())
    addFactor<CombinedImuFactor>(propagation_from_radar);
  // if (propagation_from_radar.getLastStateIdx().has_value()) {
  //   LOG(E, "Weird.");
  // }

  // Add all doppler factors to split_state.
  addDopplerFactors(propagation_to_radar, noise_model_radar_doppler,
                    doppler_residuals);

  // Add all bearing range factors to split_state.
  addFactor<BearingRangeFactor<Pose3, Point3>>(propagation_to_radar,
                                               noise_model_radar_track);

  auto idx = propagation_to_radar.graph_idx_;
  auto state = propagation_to_radar.state_;
  new_values_.insert(X(idx), state.getPose());
  new_timestamps_[X(idx)] = state.imu->header.stamp.toSec();
  new_values_.insert(V(idx), state.I_v_IB);
  new_timestamps_[V(idx)] = state.imu->header.stamp.toSec();
  new_values_.insert(B(idx), state.getBias());
  new_timestamps_[B(idx)] = state.imu->header.stamp.toSec();
  gttoc_(addRadarFactor);
  tictoc_finishedIteration_();
  tictoc_getNode(addRadarFactor, addRadarFactor);
  if (timing_.find("optimize") != timing_.end())
    updateTiming(addRadarFactor, "addRadarFactor",
                 timing_["optimize"].header.stamp);
}

void Optimization::addBaroFactor(
    const Propagation& propagation, const double baro_height,
    const double baro_height_offset,
    const gtsam::SharedNoiseModel& noise_model_baro_height,
    gtsam::Vector1* baro_residual) {
  auto idx = propagation.graph_idx_;
  auto state = propagation.state_;
  auto h = dot(Point3_(Point3(0, 0, 1)), translation(Pose3_(X(idx)))) +
           Double_(baro_height_offset);

  auto factor = ExpressionFactor(noise_model_baro_height, baro_height, h);
  new_graph_.add(factor);

  if (baro_residual) {
    Values x;
    x.insert(X(idx), state.getPose());
    *baro_residual = factor.unwhitenedError(x);
  }
}

bool Optimization::solve(const LinkedPropagations& linked_propagations) {
  if (running_.load()) {
    LOG(W, "Optimization thread still running.");
    return false;
  }
  if (thread_.joinable()) {
    LOG(W, "Optimization thread not joined, get result first.");
    return false;
  }

  running_.store(true);
  solveThreaded(new_graph_, new_values_, new_timestamps_);
  //  thread_ = std::thread(&Optimization::solveThreaded, this, new_graph_,
  //                        new_values_, new_timestamps_);

  new_graph_.resize(0);
  new_values_.clear();
  new_timestamps_.clear();
  return true;
}

bool Optimization::getResult(LinkedPropagations& linked_propagations) {
  if (!running_.load()) {
    if (thread_.joinable()) {
      thread_.join();
    }
  } else
    return false;

  if (!new_result_.load()) return false;
  gttic_(getResult);
  new_result_.store(false);
  {
    std::scoped_lock lock(values_mutex_);
    linked_propagations.head->prior->updateState(optimized_values_);
  }
  linked_propagations.head->state_ = linked_propagations.head->prior->state_;

  linked_propagations.head->repropagate();
  linked_propagations.remove(cutoff_time);

  gttoc_(getResult);
  tictoc_finishedIteration_();
  tictoc_getNode(getResult, getResult);
  updateTiming(getResult, "getResult", timing_["optimize"].header.stamp);
  return true;
}

void Optimization::solveThreaded(
    const gtsam::NonlinearFactorGraph graph, const gtsam::Values values,
    const gtsam::FixedLagSmoother::KeyTimestampMap stamps) {
  gttic_(optimize);
  try {
    smoother_.update(graph, values, stamps);
  } catch (const std::exception& e) {
    LOG(E, "Exception in update: " << e.what());
    resetSmoother();
    gttoc_(optimize);
    return;
  }
  gttoc_(optimize);

  gttic_(calculateEstimate);
  try {
    std::scoped_lock lock(values_mutex_);
    optimized_values_ = smoother_.calculateEstimate();
  } catch (const std::exception& e) {
    LOG(E, "Exception in calculateEstimate: " << e.what());
    resetSmoother();
    gttoc_(calculateEstimate);
    return;
  }
  gttoc_(calculateEstimate);

  // Update propagations.
  cutoff_time =
      std::min_element(
          smoother_.timestamps().begin(), smoother_.timestamps().end(),
          [](const auto& a, const auto& b) { return a.second < b.second; })
          ->second;
  tictoc_finishedIteration_();
  tictoc_getNode(optimize, optimize);
  updateTiming(optimize, "optimize", timing_["optimize"].header.stamp);
  tictoc_getNode(calculateEstimate, calculateEstimate);
  updateTiming(calculateEstimate, "calculateEstimate",
               timing_["optimize"].header.stamp);

  new_result_.store(true);
  running_.store(false);
}

void Optimization::resetSmoother() {
  ISAM2Params parameters = smoother_.params();
  double smoother_lag = smoother_.smootherLag();
  smoother_ = IncrementalFixedLagSmoother(smoother_lag, parameters);
  std::scoped_lock lock(values_mutex_);
  optimized_values_ = gtsam::Values();
  running_.store(false);
  smoother_failed_.store(true);
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
