#include "rio/gtsam/propagation.h"

#include <algorithm>

#include <log++.h>
#include <tf2_eigen/tf2_eigen.h>

using namespace rio;
using namespace gtsam;

Propagation::Propagation(const State& state,
                          const uint64_t idx)
                          : state_(state),
                            graph_idx_(idx) {}

// Propagation::Propagation(const State& initial_state,
//                          const uint64_t first_state_idx,
//                          const std::optional<uint64_t>& last_state_idx)
//     : Propagation(std::make_shared<State>(initial_state), first_state_idx,
//                   last_state_idx) {}

// Propagation::Propagation(const State::ConstPtr& initial_state,
//                          const uint64_t first_state_idx,
//                          const std::optional<uint64_t>& last_state_idx)
//     : Propagation(std::vector<State::ConstPtr>({initial_state}),
//                   first_state_idx, last_state_idx) {}

// Propagation::Propagation(const std::vector<State::ConstPtr>& initial_states,
//                          const uint64_t first_state_idx,
//                          const std::optional<uint64_t>& last_state_idx)
//     : states_(initial_states),
//       first_state_idx_(first_state_idx),
//       last_state_idx_(last_state_idx) {}

bool Propagation::addImuMeasurement(const sensor_msgs::Imu& msg) {
  return addImuMeasurement(sensor_msgs::ImuConstPtr(new sensor_msgs::Imu(msg)));
}

bool Propagation::addImuMeasurement(const sensor_msgs::ImuConstPtr& msg) {
  // if (states_.empty()) {
  //   LOG(E, "No initial state, skipping IMU integration.");
  //   return false;
  // }
  // if (states_.front() == nullptr) {
  //   LOG(E, "Initial state not complete, skipping IMU integration.");
  //   return false;
  // }
  // if (states_.back()->imu == nullptr) {
  //   LOG(E, "Previous IMU measurement not complete, skipping IMU integration.");
  //   return false;
  // }

  auto dt = (msg->header.stamp - state_.imu->header.stamp).toSec();
  if (dt < 0) {
    LOG(W, "Negative dt, skipping IMU integration.");
    return false;
  }
  if (dt == 0) {
    LOG(W, "Zero dt, skipping IMU integration.");
    return false;
  }
  Vector3 lin_acc, ang_vel;
  tf2::fromMsg(msg->linear_acceleration, lin_acc);
  tf2::fromMsg(msg->angular_velocity, ang_vel);



  // auto integrator = integrators_.back();
  state_.integrator.integrateMeasurement(lin_acc, ang_vel, dt);
  auto prediction =
      state_.integrator.predict(state_.getNavState(), state_.integrator.biasHat());
  state_ = State(state_.odom_frame_id, prediction.pose().translation(),
                 prediction.pose().rotation(), prediction.velocity(), msg,
                 state_.integrator);

  imu_measurements_.push_back(msg);
  // integrators_.push_back(integrator);

  // auto integrator = states_.back()->integrator;
  // integrator.integrateMeasurement(lin_acc, ang_vel, dt);
  // auto prediction =
  //     integrator.predict(states_.front()->getNavState(), integrator.biasHat());
  // states_.push_back(std::make_shared<State>(
  //     states_.back()->odom_frame_id, prediction.pose().translation(),
  //     prediction.pose().rotation(), prediction.velocity(), msg, integrator));
  return true;
}

bool Propagation::split(const ros::Time& t, uint64_t* split_idx,
                        Propagation* propagation_to_t,
                        Propagation* propagation_from_t) {
  // if (states_.empty()) {
  //   LOG(W, "No initial state, skipping split.");
  //   return false;
  // }
  // if (states_.front() == nullptr) {
  //   LOG(W, "Initial state not complete, skipping split.");
  //   return false;
  // }
  // if (t < states_.front()->imu->header.stamp) {
  //   LOG(D, "t is before first IMU measurement, skipping split.");
  //   return false;
  // }
  // if (t > states_.back()->imu->header.stamp) {
  //   LOG(D, "t is after last IMU measurement, skipping split.");
  //   return false;
  // }

  // returns first state which is not less than t
  // auto state_1 =
  //     std::lower_bound(states_.begin(), states_.end(), t,
  //                      [](const State::ConstPtr& state, const ros::Time& t) {
  //                        return state->imu->header.stamp < t;
  //                      });

  // auto state_1 =
  //   std::lower_bound(imu_measurements_.begin(), imu_measurements_.end(), t,
  //                    [](const sensor_msgs::ImuConstPtr& imu, const ros::Time&
  //                    t) {
  //                      return imu->header.stamp < t;
  //                    });

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

// if (!propagation_to_t->imu_measurements_.empty()) { //todo

//   propagation_to_t->integrators_ =
//       std::vector<gtsam::PreintegratedCombinedMeasurements>(
//           propagation_to_t->integrators_.begin(),
//           propagation_to_t->integrators_.begin() +
//               propagation_to_t->imu_measurements_.size());
//   }


  // repropagate because i updated prior with optimized states
  propagation_to_t->repropagate();

  if (t > propagation_to_t->imu_measurements_.back()->header.stamp) {

    sensor_msgs::Imu imu;
    imu.header = propagation_to_t->imu_measurements_.back()->header;
    imu.header.stamp = t;
    imu.linear_acceleration = propagation_to_t->imu_measurements_.back()->linear_acceleration;
    imu.angular_velocity = propagation_to_t->imu_measurements_.back()->angular_velocity;
    propagation_to_t->addImuMeasurement(imu);
  }
  propagation_to_t->graph_idx_ = *split_idx;
  propagation_from_t->state_ = propagation_to_t->state_;

  propagation_from_t->repropagate();

    // *propagation_to_t =
    //     Propagation(std::vector<State::ConstPtr>(states_.begin(), state_1),
    //                 first_state_idx_, (*split_idx));
    // if (t > (*state_0)->imu->header.stamp)
    //   propagation_to_t->addImuMeasurement(imu);
    // else
    //   LOG(W, "Split before or exactly at measurement time. t_split: "
    //              << t << " t_0: " << (*state_0)->imu->header.stamp);

    // for (auto it = propagation_to_t->imu_measurements_.begin();
    //      it != propagation_to_t->imu_measurements_.end(); ++it) {
    //   propagation_to_t->addImuMeasurement(*it);
    // }

    // // Regenerate propagation from t.
    // if (t < (*state_1)->imu->header.stamp) {
    //   State initial_state =
    //   {propagation_to_t->getLatestState()->odom_frame_id,
    //                          propagation_to_t->getLatestState()->I_p_IB,
    //                          propagation_to_t->getLatestState()->R_IB,
    //                          propagation_to_t->getLatestState()->I_v_IB,
    //                          propagation_to_t->getLatestState()->imu,
    //                          propagation_to_t->getLatestState()->integrator};
    //   initial_state.integrator.resetIntegrationAndSetBias(
    //       propagation_to_t->getLatestState()->integrator.biasHat());
    //   *propagation_from_t =
    //       Propagation(initial_state, (*split_idx), last_state_idx_);
    //   for (auto it = state_1; it != states_.end(); ++it) {
    //     propagation_from_t->addImuMeasurement((*it)->imu);
    //   }
    // } else {
    //   *propagation_from_t =
    //       Propagation(std::vector<State::ConstPtr>(state_1, states_.end()),
    //                   (*split_idx), last_state_idx_);
    //   LOG(W, "Split after or exactly at measurement time. t_split: "
    //              << t << " t_1: " << (*state_1)->imu->header.stamp);
    // }
    (*split_idx)++;

    return true;
}

// bool Propagation::repropagate(const State& initial_state) {
//   if (states_.empty()) {
//     LOG(W, "No initial state, skipping repropagation.");
//     return false;
//   }
//   auto first_state = initial_state;
//   first_state.integrator.resetIntegration();

//   Propagation propagation(first_state, first_state_idx_, last_state_idx_);
//   for (auto it = std::next(states_.begin()); it != states_.end(); ++it) {
//     if (!propagation.addImuMeasurement((*it)->imu)) {
//       LOG(W, "Failed to add IMU message during repropagation.");
//       return false;
//     }
//   }

//   *this = propagation;

//   return true;
// }