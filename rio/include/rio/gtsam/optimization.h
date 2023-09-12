#pragma once

#include <thread>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam_unstable/nonlinear/FixedLagSmoother.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include "rio/gtsam/state.h"

namespace rio {

class Optimization {
 public:
  Optimization(){};
  // bool solve();
  void addPriorFactor(const State::ConstPtr& state,
                      const gtsam::SharedNoiseModel& noise_model_I_T_IB,
                      const gtsam::SharedNoiseModel& noise_model_I_v_IB,
                      const gtsam::SharedNoiseModel& noise_model_imu_bias);
  void addRadarFactor(const State::ConstPtr& prev_state,
                      const State::ConstPtr& split_state,
                      const State::ConstPtr& next_state,
                      const gtsam::SharedNoiseModel& noise_model_radar);

 private:
  // void solveThreaded();

  template <typename T>
  void addFactor(const uint32_t idx, const State::ConstPtr& state,
                 const gtsam::SharedNoiseModel& noise_model);

  gtsam::IncrementalFixedLagSmoother smoother_;
  uint32_t idx_{0};

  gtsam::NonlinearFactorGraph new_graph_;
  gtsam::Values new_values_;
  gtsam::FixedLagSmoother::KeyTimestampMap new_timestamps_;

  gtsam::NonlinearFactorGraph optimized_graph_;
  gtsam::Values optimized_values_;
  gtsam::FixedLagSmoother::KeyTimestampMap optimized_timestamps_;

  std::thread thread_;
};

}  // namespace rio