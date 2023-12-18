#pragma once

#include <optional>
#include <vector>

#include <mav_sensors_drivers/sensor_types/Radar.h>

#include "rio/gtsam/landmark_tracker.h"
#include "rio/gtsam/state.h"
#include "sensor_msgs/Imu.h"
#include "mutex"
#include "ros/ros.h"
#include <gtsam/inference/Symbol.h>

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::L;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

namespace rio {

class Propagation {
 public:
  inline Propagation(){};
  Propagation(const State& state,
              const uint64_t idx);
  // Propagation(const State& initial_state, const uint64_t first_state_idx,
  //             const std::optional<uint64_t>& last_state_idx = std::nullopt);
  // Propagation(const State::ConstPtr& initial_state,
  //             const uint64_t first_state_idx,
  //             const std::optional<uint64_t>& last_state_idx = std::nullopt);
  // Propagation(const std::vector<State::ConstPtr>& initial_states,
  //             const uint64_t first_state_idx,
  //             const std::optional<uint64_t>& last_state_idx = std::nullopt);
  bool addImuMeasurement(const sensor_msgs::ImuConstPtr& msg);
  bool addImuMeasurement(const sensor_msgs::Imu& msg);
  // inline State::ConstPtr getLatestState() const { return states_.back(); }
  // inline State::ConstPtr getFirstState() const { return states_.front(); }
  bool split(const ros::Time& t, uint64_t* split_idx,
             Propagation* propagation_to_t,
             Propagation* propagation_from_t);
  // bool repropagate(const State& initial_state);

  // inline uint64_t getFirstStateIdx() const { return first_state_idx_; }
  // inline std::optional<uint64_t> getLastStateIdx() const {
  //   return last_state_idx_;
  // }

  inline u_int64_t getGraphIdx() const {
    // std::scoped_lock lock(propagation_mutex_);
    return graph_idx_; }
  inline State getState() const {
    // std::scoped_lock lock(propagation_mutex_);
    return state_; }
  

  std::optional<gtsam::Pose3> B_T_BR_;
  std::optional<std::vector<mav_sensors::Radar::CfarDetection>>
      cfar_detections_;
  std::optional<std::vector<Track::Ptr>> cfar_tracks_;

  Propagation *prior;

  // std::mutex propagation_mutex_;
  mutable std::mutex propagation_mutex_;

  State state_;
  std::vector<sensor_msgs::ImuConstPtr> imu_measurements_;
  // std::vector<gtsam::PreintegratedCombinedMeasurements> integrators_;
  u_int64_t graph_idx_{0};

  void repropagate() {
      // std::scoped_lock lock(propagation_mutex_, prior->propagation_mutex_);
      auto imu_measurements_copy = imu_measurements_;
      imu_measurements_.clear();
      // integrators_.clear();
      for (auto imu_measurement : imu_measurements_copy) {
        addImuMeasurement(imu_measurement);
      }
  }

 private:
  // Vector of IMU measurements and preintegration up to this IMU.
  std::vector<State::ConstPtr> states_;


  uint64_t first_state_idx_{0};
  std::optional<uint64_t> last_state_idx_;
};




class LinkedPropagations {
  private:

  public:
    Propagation* head;
    LinkedPropagations() : head(nullptr) {}

    void append(Propagation* new_propagation) {
      new_propagation->prior = head;
      head = new_propagation;
    }

  // call getSplitPropagation
  // then in rio: update prior Propagation to split
  // then in rio: insertPrior
    Propagation* getSplitPropagation(ros::Time t) {
      Propagation* current = head;
      while (current) {
        std::scoped_lock lock(current->propagation_mutex_, current->prior->propagation_mutex_);
        if (current->prior->state_.imu->header.stamp < t) {
          return current;
        current = current->prior;
        }
      }
      return current;
    }

    bool insertPrior(Propagation* propagation_new, Propagation* propagation_ref, ros::Time t, uint64_t& idx) {
      std::scoped_lock lock(propagation_ref->propagation_mutex_, propagation_new->propagation_mutex_);
      propagation_new->prior = propagation_ref->prior;
      propagation_ref->prior = propagation_new;

      // update propagation_new->prior with optimized states // todo
      // gtsam::PreintegratedCombinedMeasurements integrator; // todo
      // integrator add dt imu // todo
      // propagation_new->state_ = propagation_new->prior->state_;
      // propagation_new->state_ = integrator.predict(propagation_new->state_.getNavState(), integrator.biasHat());

      return propagation_ref->split(t, &idx, propagation_new, propagation_ref);

      

    }

    void updateState(Propagation* propagation, gtsam::Values& values) {
      std::scoped_lock lock(propagation->propagation_mutex_);
      auto pose = values.at<gtsam::Pose3>(
          X(propagation->graph_idx_));
      propagation->state_.I_p_IB = pose.translation();
      propagation->state_.R_IB = pose.rotation();
      propagation->state_.I_v_IB = values.at<gtsam::Vector3>(
          V(propagation->graph_idx_));
      propagation->state_.integrator.resetIntegrationAndSetBias(
          values.at<gtsam::imuBias::ConstantBias>(
              B(propagation->graph_idx_)));
    }



    void remove(double t) {
      Propagation* current = head;


      while (current->prior){
        std::scoped_lock lock(current->propagation_mutex_, current->prior->propagation_mutex_);
        if (current->prior->state_.imu->header.stamp.toSec() < t) {
          break;
        }
        current = current->prior;
      }
      
      if (current->prior) {
        auto to_delete = current->prior;
        current->prior = nullptr;
        Propagation* priorProp = nullptr;

        while (to_delete) {
          // lock not needed. only delete propagations at the end of the list
          // where no other thread would access
          priorProp = to_delete->prior;
          delete to_delete;
          to_delete = priorProp;
        }
      }
    }

};


}  // namespace rio