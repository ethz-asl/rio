#include "rio/gtsam/propagation.h"

#include <algorithm>

#include <log++.h>
#include <tf2_eigen/tf2_eigen.h>

using namespace rio;
using namespace gtsam;

Propagation::Propagation(const State& state, const uint64_t idx)
    : state_(state), graph_idx_(idx) {}

bool Propagation::addImuMeasurement(const sensor_msgs::Imu& msg) {
  return addImuMeasurement(sensor_msgs::ImuConstPtr(new sensor_msgs::Imu(msg)));
}

bool Propagation::addImuMeasurement(const sensor_msgs::ImuConstPtr& msg) {
  auto dt = (msg->header.stamp - state_.imu->header.stamp).toSec();
  if (dt < 0) {
    LOG(W, "Negative dt, skipping IMU integration.");
    return false;
  }
  if (dt == 0) {
    LOG(W, "Zero dt, skipping IMU integration.");
    return false;
  }
  propagate_imu(msg, dt, state_.integrator);
  predict_state(msg);

  imu_measurements_.push_back(msg);

  return true;
}

bool Propagation::split(const ros::Time& t, uint64_t* split_idx,
                        Propagation* propagation_to_t,
                        Propagation* propagation_from_t) {
  auto partition_iter =
      std::partition(imu_measurements_.begin(), imu_measurements_.end(),
                     [&t](const sensor_msgs::ImuConstPtr& imu) {
                       return imu->header.stamp <= t;
                     });
  propagation_to_t->imu_measurements_ = std::vector<sensor_msgs::ImuConstPtr>(
      imu_measurements_.begin(), partition_iter);
  if (propagation_to_t->imu_measurements_.empty()) {
    LOG(W, "No IMU measurements in propagation to t, skipping split.");
    return false;
  }
  propagation_from_t->imu_measurements_ = std::vector<sensor_msgs::ImuConstPtr>(
      partition_iter, imu_measurements_.end());

  // LOG(I, "nr imu to t: " << propagation_to_t->imu_measurements_.size()
  //                        << " nr imu from t: "
  //                        << propagation_from_t->imu_measurements_.size());

  // repropagate because i updated prior with optimized states
  // TODO: not necessary if last state contains already optimized state

  propagation_to_t->repropagate();

  if (t > propagation_to_t->imu_measurements_.back()->header.stamp) {
    sensor_msgs::Imu imu;
    imu.header = propagation_to_t->imu_measurements_.back()->header;
    imu.header.stamp = t;
    imu.linear_acceleration =
        propagation_to_t->imu_measurements_.back()->linear_acceleration;
    imu.angular_velocity =
        propagation_to_t->imu_measurements_.back()->angular_velocity;
    propagation_to_t->addImuMeasurement(imu);
  }

  propagation_to_t->graph_idx_ = *split_idx;
  propagation_from_t->state_ = propagation_to_t->state_;

  if (!propagation_from_t->imu_measurements_.empty() &&
      t < propagation_from_t->imu_measurements_.front()->header.stamp) {
    sensor_msgs::Imu imu;
    imu.header = propagation_from_t->imu_measurements_.front()->header;
    imu.header.stamp = t;
    imu.linear_acceleration =
        propagation_from_t->imu_measurements_.front()->linear_acceleration;
    imu.angular_velocity =
        propagation_from_t->imu_measurements_.front()->angular_velocity;
    propagation_from_t->imu_measurements_.insert(
        propagation_from_t->imu_measurements_.begin(),
        sensor_msgs::ImuConstPtr(new sensor_msgs::Imu(imu)));
  } else {
    sensor_msgs::Imu imu;
    imu.header = propagation_to_t->imu_measurements_.back()->header;
    imu.header.stamp = ros::Time(t.toSec() + 0.0001);
    imu.linear_acceleration =
        propagation_to_t->imu_measurements_.back()->linear_acceleration;
    imu.angular_velocity =
        propagation_to_t->imu_measurements_.back()->angular_velocity;
    propagation_from_t->imu_measurements_.push_back(
        sensor_msgs::ImuConstPtr(new sensor_msgs::Imu(imu)));
  }

  propagation_from_t->repropagate();

  (*split_idx)++;

  return true;
}

void Propagation::repropagate() {
  for (auto it = imu_measurements_.begin() + 1; it != imu_measurements_.end();
       ++it) {
    auto dt = ((*it)->header.stamp - (*(it - 1))->header.stamp).toSec();
    propagate_imu(*it, dt, state_.integrator);
  }
  predict_state(imu_measurements_.back());
}

void Propagation::propagate_imu(
    const sensor_msgs::ImuConstPtr& msg, double dt,
    gtsam::PreintegratedCombinedMeasurements& integrator) {
  gtsam::Vector3 lin_acc, ang_vel;
  tf2::fromMsg(msg->linear_acceleration, lin_acc);
  tf2::fromMsg(msg->angular_velocity, ang_vel);
  integrator.integrateMeasurement(lin_acc, ang_vel, dt);
}

void Propagation::predict_state(const sensor_msgs::ImuConstPtr& msg) {
  auto prediction = state_.integrator.predict(prior->state_.getNavState(),
                                              state_.integrator.biasHat());
  // state_.integrator.predict(state_.getNavState(),
  // state_.integrator.biasHat());
  state_ = State(state_.odom_frame_id, prediction.pose().translation(),
                 prediction.pose().rotation(), prediction.velocity(), msg,
                 state_.integrator);
}

void Propagation::updateState(gtsam::Values& values) {
  try {
    auto pose = values.at<gtsam::Pose3>(X(graph_idx_));
    state_.I_p_IB = pose.translation();
    state_.R_IB = pose.rotation();
    state_.I_v_IB = values.at<gtsam::Vector3>(V(graph_idx_));
    state_.integrator.resetIntegrationAndSetBias(
        values.at<gtsam::imuBias::ConstantBias>(B(graph_idx_)));
  } catch (const std::exception& e) {
    LOG(W, "Could not update state: " << e.what());
  }
}