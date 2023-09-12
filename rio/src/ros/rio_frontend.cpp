#include "rio/ros/rio_frontend.h"

#include <cmath>

#include <log++.h>
#include <nav_msgs/Odometry.h>
#include <tf2_eigen/tf2_eigen.h>

#include "rio/ros/common.h"

using namespace rio;
using namespace gtsam;

RioFrontend::RioFrontend(const ros::NodeHandle& nh,
                         const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {}

bool RioFrontend::init() {
  // ROS communication.
  int queue_size = 1;
  if (!loadParam<int>(nh_private_, "queue_size", &queue_size)) return false;

  // Subscribers.
  imu_raw_sub_ = nh_.subscribe("imu/data_raw", queue_size,
                               &RioFrontend::imuRawCallback, this);
  imu_filter_sub_ = nh_.subscribe("imu/data", queue_size,
                                  &RioFrontend::imuFilterCallback, this);
  radar_cfar_sub_ = nh_.subscribe("radar/cfar_detections", queue_size,
                                  &RioFrontend::cfarDetectionsCallback, this);

  // Publishers
  odom_navigation_pub_ = nh_private_.advertise<nav_msgs::Odometry>(
      "odometry_navigation", queue_size);
  odom_optimizer_pub_ = nh_private_.advertise<nav_msgs::Odometry>(
      "odometry_optimizer", queue_size);

  // IMU integration
  double bias_acc_sigma = 0.0, bias_omega_sigma = 0.0, bias_acc_int_sigma = 0.0,
         bias_omega_int_sigma = 0.0, acc_sigma = 0.0, integration_sigma = 0.0,
         gyro_sigma = 0.0;
  // TODO: Possibly expose "use2ndOrderCoriolis", "omegaCoriolis", "n_gravity"
  // or "body_P_sensor" as parameters. But only really makes sense if we have a
  // earth-centered coordinate frame or not a IMU centered frame.
  if (!loadParam<double>(nh_private_, "imu/bias_acc_sigma", &bias_acc_sigma))
    return false;
  if (!loadParam<double>(nh_private_, "imu/bias_omega_sigma",
                         &bias_omega_sigma))
    return false;
  if (!loadParam<double>(nh_private_, "imu/bias_acc_int_sigma",
                         &bias_acc_int_sigma))
    return false;
  if (!loadParam<double>(nh_private_, "imu/bias_omega_int_sigma",
                         &bias_omega_int_sigma))
    return false;
  if (!loadParam<double>(nh_private_, "imu/acc_sigma", &acc_sigma))
    return false;
  if (!loadParam<double>(nh_private_, "imu/integration_sigma",
                         &integration_sigma))
    return false;
  if (!loadParam<double>(nh_private_, "imu/gyro_sigma", &gyro_sigma))
    return false;

  auto imu_params = PreintegratedCombinedMeasurements::Params::MakeSharedU();
  imu_params->biasAccCovariance = I_3x3 * std::pow(bias_acc_sigma, 2);
  imu_params->biasOmegaCovariance = I_3x3 * std::pow(bias_omega_sigma, 2);
  imu_params->biasAccOmegaInt.block<3, 3>(0, 0) =
      I_3x3 * std::pow(bias_acc_int_sigma, 2);
  imu_params->biasAccOmegaInt.block<3, 3>(3, 3) =
      I_3x3 * std::pow(bias_omega_int_sigma, 2);

  imu_params->accelerometerCovariance = I_3x3 * std::pow(acc_sigma, 2);
  imu_params->integrationCovariance = I_3x3 * std::pow(integration_sigma, 2);
  imu_params->gyroscopeCovariance = I_3x3 * std::pow(gyro_sigma, 2);

  Vector3 b_a, b_g;
  if (!loadParam<Vector3>(nh_private_, "imu/initial_bias_acc", &b_a))
    return false;
  if (!loadParam<Vector3>(nh_private_, "imu/initial_bias_gyro", &b_g))
    return false;
  imu_params->print("IMU parameters:");
  PreintegratedCombinedMeasurements integrator{imu_params, {b_a, b_g}};
  integrator.print("Initial preintegration parameters:");

  // Initial state.
  std::string odom_frame_id = initial_state_->odom_frame_id;
  loadParam<std::string>(nh_private_, "odom_frame_id", &odom_frame_id);

  initial_state_ = std::make_shared<State>(
      odom_frame_id, initial_state_->I_p_IB, initial_state_->R_IB,
      initial_state_->I_v_IB, initial_state_->imu, integrator);

  // Prior noise pose.
  Vector3 prior_noise_R_IB, prior_noise_I_p_IB;
  if (!loadParam<Vector3>(nh_private_, "prior_noise/R_IB", &prior_noise_R_IB))
    return false;
  if (!loadParam<Vector3>(nh_private_, "prior_noise/I_p_IB",
                          &prior_noise_I_p_IB))
    return false;
  prior_noise_model_I_T_IB_ = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << prior_noise_R_IB, prior_noise_I_p_IB).finished());
  prior_noise_model_I_T_IB_->print("prior_noise_model_I_T_IB: ");

  // Prior noise velocity.
  Vector3 prior_noise_I_v_IB;
  if (!loadParam<Vector3>(nh_private_, "prior_noise/I_v_IB",
                          &prior_noise_I_v_IB))
    return false;
  prior_noise_model_I_v_IB_ =
      gtsam::noiseModel::Diagonal::Sigmas(prior_noise_I_v_IB);
  prior_noise_model_I_v_IB_->print("prior_noise_model_I_v_IB: ");

  // Prior noise IMU bias.
  Vector3 prior_noise_bias_acc, prior_noise_bias_gyro;
  if (!loadParam<Vector3>(nh_private_, "prior_noise/b_a",
                          &prior_noise_bias_acc))
    return false;
  if (!loadParam<Vector3>(nh_private_, "prior_noise/b_g",
                          &prior_noise_bias_gyro))
    return false;
  prior_noise_model_imu_bias_ = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << prior_noise_bias_acc, prior_noise_bias_gyro)
          .finished());
  prior_noise_model_imu_bias_->print("prior_noise_model_imu_bias: ");

  double max_dead_reckoning_duration = max_dead_reckoning_duration_.toSec();
  if (!loadParam<double>(nh_private_, "max_dead_reckoning_duration",
                         &max_dead_reckoning_duration))
    return false;
  max_dead_reckoning_duration_ = ros::Duration(max_dead_reckoning_duration);
  return true;
}

void RioFrontend::imuRawCallback(const sensor_msgs::ImuConstPtr& msg) {
  LOG_FIRST(I, 1, "Received first raw IMU message.");
  // Initialize.
  if (initial_state_->imu == nullptr) {
    LOG_TIMED(W, 1.0, "Initial state not complete, skipping IMU integration.");
    return;
  } else if (propagation_.empty()) {
    LOG(I, "Initializing states with initial state.");
    propagation_.emplace_back(initial_state_, idx_++);
    optimization_.addPriorFactor(initial_state_, prior_noise_model_I_T_IB_,
                                 prior_noise_model_I_v_IB_,
                                 prior_noise_model_imu_bias_);
    return;
  }
  // Integrate.
  if (!propagation_.back().addImuMeasurement(msg)) {
    LOG(W, "Failed to add IMU measurement, skipping IMU integration.");
    return;
  }
  // Publish.
  odom_navigation_pub_.publish(
      propagation_.back().getLatestState()->getOdometry());
}

void RioFrontend::imuFilterCallback(const sensor_msgs::ImuConstPtr& msg) {
  LOG_FIRST(I, 1, "Received first filtered IMU message.");
  Eigen::Quaterniond q_IB;
  tf2::fromMsg(msg->orientation, q_IB);
  initial_state_ = std::make_shared<State>(
      initial_state_->odom_frame_id, initial_state_->I_p_IB, Rot3(q_IB),
      initial_state_->I_v_IB, msg, initial_state_->integrator);
}

void RioFrontend::cfarDetectionsCallback(const sensor_msgs::PointCloud2& msg) {
  LOG_FIRST(I, 1, "Received first CFAR detections.");
  if (propagation_.empty()) {
    LOG(W, "No propagation, skipping CFAR detections.");
    return;
  }

  if (!splitPropagation(msg.header.stamp)) {
    LOG(W, "Failed to split propagation, skipping CFAR detections.");
    LOG(W, "Split time: " << msg.header.stamp);
    LOG(W, "Last IMU time: "
               << propagation_.back().getLatestState()->imu->header.stamp);
    return;
  }

  popOldPropagations();
}

bool RioFrontend::splitPropagation(const ros::Time& t) {
  bool success = false;
  for (auto it = propagation_.begin(); it != propagation_.end(); ++it) {
    Propagation propagation_to_t, propagation_from_t;
    success = it->split(t, &idx_, &propagation_to_t, &propagation_from_t);
    if (success) {
      *it = propagation_to_t;
      propagation_.insert(std::next(it), propagation_from_t);
      break;
    }
  }

  return success;
}

void RioFrontend::popOldPropagations() {
  auto now = ros::Time::now();
  while (propagation_.size() > 1 &&
         (now - propagation_.front().getLatestState()->imu->header.stamp) >
             max_dead_reckoning_duration_) {
    propagation_.pop_front();
  }
}