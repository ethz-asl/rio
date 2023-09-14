#pragma once

#include <deque>
#include <thread>
#include <map>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam_unstable/nonlinear/FixedLagSmoother.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include "rio/Timing.h"
#include "rio/gtsam/propagation.h"

namespace rio {

class Optimization {
 public:
  Optimization(){};
  bool solve(const std::deque<Propagation>& propagations);
  bool getResult(std::deque<Propagation>* propagation, std::map<std::string, Timing>* timing);

  void addPriorFactor(const Propagation& propagation,
                      const gtsam::SharedNoiseModel& noise_model_I_T_IB,
                      const gtsam::SharedNoiseModel& noise_model_I_v_IB,
                      const gtsam::SharedNoiseModel& noise_model_imu_bias);
  void addRadarFactor(const Propagation& propagation_to_radar,
                      const Propagation& propagation_from_radar,
                      const gtsam::SharedNoiseModel& noise_model_radar);
  inline void setSmoother(const gtsam::IncrementalFixedLagSmoother& smoother) {
    smoother_ = smoother;
  }

 private:
  void solveThreaded(const gtsam::NonlinearFactorGraph& graph,
                     const gtsam::Values& values,
                     const gtsam::FixedLagSmoother::KeyTimestampMap& stamps);

  template <typename T>
  void addFactor(const Propagation& propagation,
                 const gtsam::SharedNoiseModel& noise_model = nullptr);

  void updateTiming(const std::string& name);

  gtsam::NonlinearFactorGraph new_graph_;
  gtsam::Values new_values_;
  gtsam::FixedLagSmoother::KeyTimestampMap new_timestamps_;

  // Variables that should not be accessed while thread is running.
  // TODO(rikba): Possibly mutex lock.
  gtsam::IncrementalFixedLagSmoother smoother_;
  std::map<std::string, Timing> timing_;
  std::deque<Propagation> propagations_;
  bool new_result_{false};

  std::thread thread_;
};

}  // namespace rio