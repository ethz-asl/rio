#pragma once

#include "mutex"
#include <optional>
#include <vector>

#include <gtsam/inference/Symbol.h>
#include <mav_sensors_drivers/sensor_types/Radar.h>
#include <tf2_eigen/tf2_eigen.h>

#include "rio/gtsam/landmark_tracker.h"
#include "rio/gtsam/state.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::L;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

namespace rio {

class Propagation {
 public:
  inline Propagation(){};
  Propagation(const State& state, const uint64_t idx);
  bool addImuMeasurement(const sensor_msgs::ImuConstPtr& msg);
  bool addImuMeasurement(const sensor_msgs::Imu& msg);
  bool split(const ros::Time& t, uint64_t* split_idx,
             Propagation* propagation_to_t);
  void repropagate();
  void propagate_imu(const sensor_msgs::ImuConstPtr& msg, double dt,
                     gtsam::PreintegratedCombinedMeasurements& integrator);
  void predict_state(const sensor_msgs::ImuConstPtr& msg);
  void updateState(gtsam::Values& values);

  std::optional<gtsam::Pose3> B_T_BR_;
  std::optional<std::vector<mav_sensors::Radar::CfarDetection>>
      cfar_detections_;
  std::optional<std::vector<Track::Ptr>> cfar_tracks_;
  Propagation* prior;

  State state_;
  std::vector<sensor_msgs::ImuConstPtr> imu_measurements_;
  u_int64_t graph_idx_{0};

};

class LinkedPropagations {
 private:
 public:
  Propagation* head;
  LinkedPropagations() : head(nullptr) {}

  Propagation* getSplitPropagation(ros::Time t) {
    Propagation* current = head;
    while (current->prior) {
      if (current->prior->state_.imu->header.stamp < t) {
        return current;
      }
      current = current->prior;
    }
    return nullptr;
  }

  bool insertPrior(Propagation* propagation_new, Propagation* propagation_ref,
                   ros::Time t, uint64_t& idx) {
    propagation_new->prior = propagation_ref->prior;
    propagation_ref->prior = propagation_new;
    return propagation_ref->split(t, &idx, propagation_new);
  }

  void remove(double t) {
    Propagation* current = head;

    while (current->prior) {
      if (current->prior->state_.imu->header.stamp.toSec() < t) {
        break;
      }
      current = current->prior;
    }

    // ensure that head always has an existing prior
    if (current == head) {
      current = head->prior;
    }

    if (current->prior) {
      auto to_delete = current->prior;
      current->prior = nullptr;
      Propagation* priorProp = nullptr;

      while (to_delete) {
        priorProp = to_delete->prior;
        delete to_delete;
        to_delete = priorProp;
      }
    }
  }
};

}  // namespace rio