#pragma once

#include <atomic>
#include <deque>
#include <map>
#include <mutex>
#include <thread>
#include <utility>

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
  bool getResult(std::deque<Propagation>* propagation,
                 std::map<std::string, Timing>* timing);

  void addPriorFactor(const Propagation& propagation,
                      const gtsam::SharedNoiseModel& noise_model_I_T_IB,
                      const gtsam::SharedNoiseModel& noise_model_I_v_IB,
                      const gtsam::SharedNoiseModel& noise_model_imu_bias);
  void addRadarFactor(const Propagation& propagation_to_radar,
                      const Propagation& propagation_from_radar,
                      const gtsam::SharedNoiseModel& noise_model_radar_doppler,
                      const gtsam::SharedNoiseModel& noise_model_radar_track,
                      std::vector<gtsam::Vector1>* doppler_residuals = nullptr);
  inline void setSmoother(const gtsam::IncrementalFixedLagSmoother& smoother) {
    smoother_ = smoother;
  }

 private:
  void solveThreaded(const gtsam::NonlinearFactorGraph graph,
                     const gtsam::Values values,
                     const gtsam::FixedLagSmoother::KeyTimestampMap stamps,
                     std::deque<Propagation> propagations);

  template <typename T>
  void addFactor(const Propagation& propagation,
                 const gtsam::SharedNoiseModel& noise_model = nullptr);

  void addDopplerFactors(const Propagation& propagation,
                         const gtsam::SharedNoiseModel& noise_model = nullptr,
                         std::vector<gtsam::Vector1>* doppler_residuals = nullptr);

  void updateTiming(
      const std::shared_ptr<const ::gtsam::internal::TimingOutline>& variable,
      const std::string& label, const ros::Time& stamp);

  gtsam::NonlinearFactorGraph new_graph_;
  gtsam::Values new_values_;
  gtsam::FixedLagSmoother::KeyTimestampMap new_timestamps_;

  // Variables that should not be accessed while thread is running.
  // Mutex lock!
  std::map<std::string, Timing> timing_;
  bool new_result_{false};
  std::deque<Propagation> propagations_;

  std::atomic<bool> running_{false};
  std::thread thread_;
  std::mutex mutex_;

  // The smoother must not be changed while the thread is running.
  gtsam::IncrementalFixedLagSmoother smoother_;
};

}  // namespace rio