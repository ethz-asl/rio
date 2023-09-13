#pragma once

#include <memory>
#include <thread>

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
  bool solve();
  bool getResult(Timing* timing);

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
  void solveThreaded(
      std::unique_ptr<gtsam::NonlinearFactorGraph> graph,
      std::unique_ptr<gtsam::Values> values,
      std::unique_ptr<gtsam::FixedLagSmoother::KeyTimestampMap> stamps);

  template <typename T>
  void addFactor(const Propagation& propagation,
                 const gtsam::SharedNoiseModel& noise_model = nullptr);

  gtsam::NonlinearFactorGraph new_graph_;
  gtsam::Values new_values_;
  gtsam::FixedLagSmoother::KeyTimestampMap new_timestamps_;

  // Variables only modified in the optimization thread.
  gtsam::IncrementalFixedLagSmoother smoother_;
  // Variables that should not be accessed while thread is running. Possibly
  // mutex lock.
  Timing timing_;
  bool new_result_{false};

  std::thread thread_;
};

}  // namespace rio