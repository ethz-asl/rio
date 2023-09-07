#include "rio/ros/rio_frontend.h"

#include <cmath>

#include <gtsam/navigation/CombinedImuFactor.h>
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
  radar_trigger_sub_ = nh_.subscribe("radar/trigger", queue_size,
                                     &RioFrontend::radarTriggerCallback, this);
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

  auto imu_params_ = PreintegratedCombinedMeasurements::Params::MakeSharedU();
  imu_params_->biasAccCovariance = I_3x3 * std::pow(bias_acc_sigma, 2);
  imu_params_->biasOmegaCovariance = I_3x3 * std::pow(bias_omega_sigma, 2);
  imu_params_->biasAccOmegaInt.block<3, 3>(0, 0) =
      I_3x3 * std::pow(bias_acc_int_sigma, 2);
  imu_params_->biasAccOmegaInt.block<3, 3>(3, 3) =
      I_3x3 * std::pow(bias_omega_int_sigma, 2);

  imu_params_->accelerometerCovariance = I_3x3 * std::pow(acc_sigma, 2);
  imu_params_->integrationCovariance = I_3x3 * std::pow(integration_sigma, 2);
  imu_params_->gyroscopeCovariance = I_3x3 * std::pow(gyro_sigma, 2);

  if (!loadParam<std::optional<Vector3>>(nh_private_, "imu/initial_bias_acc",
                                         &initial_state_.b_a))
    return false;
  if (!loadParam<std::optional<Vector3>>(nh_private_, "imu/initial_bias_gyro",
                                         &initial_state_.b_g))
    return false;
  imu_params_->print("IMU parameters:");
  integrator_ = PreintegratedCombinedMeasurements(
      imu_params_, {initial_state_.b_a.value(), initial_state_.b_g.value()});
  integrator_.print("Initial preintegration parameters:");

  // Initial state.
  initial_state_.odom_frame_id = "odom";
  loadParam<std::string>(nh_private_, "odom_frame_id",
                         &initial_state_.odom_frame_id.value());

  return true;
}

void RioFrontend::imuRawCallback(const sensor_msgs::ImuConstPtr& msg) {
  LOG_FIRST(I, 1, "Received first raw IMU message.");
  // Initialize.
  if (!initial_state_.isComplete()) {
    LOG_TIMED(W, 1.0, "Initial state not complete, skipping IMU integration.");
    return;
  } else if (!optimized_state_.isComplete()) {
    LOG(I, "Initializing state.");
    optimized_state_ = initial_state_;
    navigation_state_ = initial_state_;
    return;
  }
  // Integrate.
  Vector3 lin_acc, ang_vel;
  tf2::fromMsg(msg->linear_acceleration, lin_acc);
  tf2::fromMsg(msg->angular_velocity, ang_vel);
  auto dt = (msg->header.stamp - navigation_state_.stamp.value()).toSec();
  if (dt < 0) {
    LOG(W, "Negative dt, skipping IMU integration.");
    return;
  }
  integrator_.integrateMeasurement(lin_acc, ang_vel, dt);
  // Publish.
  auto prediction = integrator_.predict(optimized_state_.getNavState(),
                                        optimized_state_.getBias());
  navigation_state_ = State({.stamp = msg->header.stamp,
                             .odom_frame_id = optimized_state_.odom_frame_id,
                             .body_frame_id = optimized_state_.body_frame_id,
                             .I_p_IB = prediction.pose().translation(),
                             .R_IB = prediction.pose().rotation(),
                             .I_v_IB = prediction.velocity(),
                             .b_a = optimized_state_.b_a,
                             .b_g = optimized_state_.b_g});
  odom_navigation_pub_.publish(navigation_state_.getOdometry());
}

void RioFrontend::imuFilterCallback(const sensor_msgs::ImuConstPtr& msg) {
  LOG_FIRST(I, 1, "Received first filtered IMU message.");
  Eigen::Quaterniond q_IB;
  tf2::fromMsg(msg->orientation, q_IB);
  initial_state_.R_IB = Rot3(q_IB);
  initial_state_.stamp = msg->header.stamp;
  initial_state_.body_frame_id = msg->header.frame_id;
}

void RioFrontend::radarTriggerCallback(const std_msgs::HeaderConstPtr& msg) {
  LOG_FIRST(I, 1, "Received first radar trigger message.");
}

void RioFrontend::cfarDetectionsCallback(const sensor_msgs::PointCloud2& msg) {
  LOG_FIRST(I, 1, "Received first CFAR detections.");
}