#include "rio/ros/rio.h"

#include <cmath>

#include <gtsam/navigation/CombinedImuFactor.h>
#include <log++.h>
#include <nav_msgs/Odometry.h>
#include <tf2_eigen/tf2_eigen.h>

#include "rio/ros/common.h"

using namespace rio;
using namespace gtsam;

Rio::Rio(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {}

Rio::~Rio() {}

bool Rio::init() {
  // ROS communication.
  int queue_size = 1;
  if (!loadParam<int>(nh_private_, "queue_size", &queue_size)) return false;

  // Subscribers.
  imu_raw_sub_ =
      nh_.subscribe("imu/data_raw", queue_size, &Rio::imuRawCallback, this);
  imu_filter_sub_ =
      nh_.subscribe("imu/data", queue_size, &Rio::imuFilterCallback, this);
  radar_trigger_sub_ = nh_.subscribe("radar/trigger", queue_size,
                                     &Rio::radarTriggerCallback, this);
  radar_cfar_sub_ = nh_.subscribe("radar/cfar_detections", queue_size,
                                  &Rio::cfarDetectionsCallback, this);

  // Publishers
  odom_integrated_pub_ = nh_private_.advertise<nav_msgs::Odometry>(
      "odometry_integrated", queue_size);
  odom_optimized_pub_ = nh_private_.advertise<nav_msgs::Odometry>(
      "odometry_optimized", queue_size);

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

void Rio::imuRawCallback(const sensor_msgs::ImuConstPtr& msg) {
  LOG_FIRST(I, 1, "Received first raw IMU message.");
  // Initialization.
  if (!initial_state_.isComplete()) {
    LOG_TIMED(W, 1.0, "Initial state not complete, skipping IMU integration.");
    return;
  } else if (!state_.isComplete()) {
    LOG(I, "Initializing state.");
    state_ = initial_state_;
    return;
  }
  // Integration.
  Vector3 lin_acc, ang_vel;
  tf2::fromMsg(msg->linear_acceleration, lin_acc);
  tf2::fromMsg(msg->angular_velocity, ang_vel);
  auto dt = (msg->header.stamp - state_.stamp.value()).toSec();
  if (dt < 0) {
    LOG(W, "Negative dt, skipping IMU integration.");
    return;
  }
  integrator_.integrateMeasurement(lin_acc, ang_vel, dt);
  // Publish.
  auto nav_state = integrator_.predict(
      {state_.q_IB.value(), state_.I_p_IB.value(), state_.I_v_IB.value()},
      {state_.b_a.value(), state_.b_g.value()});
}

void Rio::imuFilterCallback(const sensor_msgs::ImuConstPtr& msg) {
  LOG_FIRST(I, 1, "Received first filtered IMU message.");
  Eigen::Quaterniond q_IB;
  tf2::fromMsg(msg->orientation, q_IB);
  initial_state_.q_IB = Rot3(q_IB);
  initial_state_.stamp = msg->header.stamp;
  initial_state_.body_frame_id = msg->header.frame_id;
}

void Rio::radarTriggerCallback(const std_msgs::HeaderConstPtr& msg) {
  LOG_FIRST(I, 1, "Received first radar trigger message.");
}

void Rio::cfarDetectionsCallback(const sensor_msgs::PointCloud2& msg) {
  LOG_FIRST(I, 1, "Received first CFAR detections.");
}

bool Rio::State::isComplete() const {
  return stamp.has_value() && odom_frame_id.has_value() &&
         body_frame_id.has_value() && I_p_IB.has_value() && q_IB.has_value() &&
         I_v_IB.has_value() && b_a.has_value() && b_g.has_value();
}

bool Rio::State::reset() {
  I_p_IB.reset();
  q_IB.reset();
  I_v_IB.reset();
  b_a.reset();
  b_g.reset();
  return true;
}

nav_msgs::Odometry Rio::State::getOdometry() const {
  nav_msgs::Odometry odom;
  if (!isComplete()) {
    LOG(W, "State not complete, returning empty odometry.");
    return odom;
  }
  odom.header.stamp = stamp.value();
  odom.header.frame_id = odom_frame_id.value();
  odom.child_frame_id = body_frame_id.value();
  odom.pose.pose.orientation = tf2::toMsg(q_IB.value().toQuaternion());
  odom.pose.pose.position = tf2::toMsg(I_p_IB.value());
  tf2::toMsg(q_IB.value().unrotate(I_v_IB.value()), odom.twist.twist.linear);

  return odom;
}

geometry_msgs::TransformStamped Rio::State::getTransform() const {
  geometry_msgs::TransformStamped transform;
  if (!isComplete()) {
    LOG(W, "State not complete, returning empty transform.");
    return transform;
  }

  transform.header.stamp = stamp.value();
  transform.header.frame_id = odom_frame_id.value();
  transform.child_frame_id = body_frame_id.value();
  transform.transform.rotation = tf2::toMsg(q_IB.value().toQuaternion());
  tf2::toMsg(I_p_IB.value(), transform.transform.translation);
  return transform;
}

geometry_msgs::Vector3Stamped Rio::State::getBiasAcc() const {
  geometry_msgs::Vector3Stamped bias_acc;
  if (!isComplete()) {
    LOG(W, "State not complete, returning empty bias acc.");
    return bias_acc;
  }
  bias_acc.header.stamp = stamp.value();
  bias_acc.header.frame_id = body_frame_id.value();
  tf2::toMsg(b_a.value(), bias_acc.vector);
  return bias_acc;
}

geometry_msgs::Vector3Stamped Rio::State::getBiasGyro() const {
  geometry_msgs::Vector3Stamped bias_gyro;
  if (!isComplete()) {
    LOG(W, "State not complete, returning empty bias gyro.");
    return bias_gyro;
  }
  bias_gyro.header.stamp = stamp.value();
  bias_gyro.header.frame_id = body_frame_id.value();
  tf2::toMsg(b_g.value(), bias_gyro.vector);
  return bias_gyro;
}
