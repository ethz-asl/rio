#include "rio/ros/rio_frontend.h"

#include <cmath>

#include <log++.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/point_cloud2_iterator.h>
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
  timing_pub_ = nh_private_.advertise<rio::Timing>("timing", 100);

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

  // Noise Radar doppler.
  double noise_radar_doppler = 0.0;
  if (!loadParam<double>(nh_private_, "noise/radar/doppler",
                         &noise_radar_doppler))
    return false;
  noise_model_radar_doppler_ = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(1) << noise_radar_doppler).finished());
  noise_model_radar_doppler_->print("noise_model_radar_doppler: ");

  Vector3 noise_radar_track;
  if (!loadParam<Vector3>(nh_private_, "noise/radar/track", &noise_radar_track))
    return false;
  noise_model_radar_track_ =
      gtsam::noiseModel::Diagonal::Sigmas(noise_radar_track);
  noise_model_radar_track_->print("noise_model_radar_track: ");

  double noise_radar_track_prior = 0.0;
  if (!loadParam<double>(nh_private_, "noise/radar/track_prior",
                         &noise_radar_track_prior))
    return false;
  noise_model_radar_track_prior_ =
      gtsam::noiseModel::Isotropic::Sigma(3, noise_radar_track_prior);
  noise_model_radar_track_prior_->print("noise_model_radar_track_prior: ");

  // Radar tracker.
  int track_age;
  if (!loadParam<int>(nh_private_, "radar/track_age", &track_age)) return false;
  tracker_ = Tracker(track_age);

  // iSAM2 smoother.
  ISAM2Params parameters;
  double relinearize_threshold;
  if (!loadParam<double>(nh_private_, "isam2/relinearize_threshold",
                         &relinearize_threshold))
    return false;
  parameters.relinearizeThreshold = relinearize_threshold;
  if (!loadParam(nh_private_, "isam2/relinearize_skip",
                 &parameters.relinearizeSkip))
    return false;
  if (!loadParam(nh_private_, "isam2/enable_partial_relinarization_check",
                 &parameters.enablePartialRelinearizationCheck))
    return false;
  if (!loadParam(nh_private_, "isam2/cache_linearized_factors",
                 &parameters.cacheLinearizedFactors))
    return false;
  if (!loadParam(nh_private_, "isam2/find_unused_factor_slots",
                 &parameters.findUnusedFactorSlots))
    return false;
  int optimizer_type = 0;
  if (!loadParam(nh_private_, "isam2/optimizer", &optimizer_type)) return false;
  switch (optimizer_type) {
    case 0:
      double wildfire_threshold;
      if (!loadParam(nh_private_, "isam2/gn/wildfire_threshold",
                     &wildfire_threshold))
        return false;
      parameters.optimizationParams =
          gtsam::ISAM2GaussNewtonParams(wildfire_threshold);
      break;
    case 1:
      parameters.optimizationParams = gtsam::ISAM2DoglegParams();
      break;
    default:
      LOG(F, "Unknown optimizer type: " << optimizer_type);
      return false;
  }

  double smoother_lag = 0.0;
  if (!loadParam(nh_private_, "isam2/smoother_lag", &smoother_lag))
    return false;
  optimization_.setSmoother({smoother_lag, parameters});

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
    optimization_.addPriorFactor(propagation_.back(), prior_noise_model_I_T_IB_,
                                 prior_noise_model_I_v_IB_,
                                 prior_noise_model_imu_bias_);
    return;
  }

  // Get update from optimization.
  std::map<std::string, Timing> timing;
  auto new_result = optimization_.getResult(&propagation_, &timing);

  // Integrate.
  if (!propagation_.back().addImuMeasurement(msg)) {
    LOG(W, "Failed to add IMU measurement, skipping IMU integration.");
    return;
  }
  // Publish.
  odom_navigation_pub_.publish(
      propagation_.back().getLatestState()->getOdometry());

  if (new_result) {
    for (const auto& time : timing) timing_pub_.publish(time.second);
    tf_broadcaster_.sendTransform(
        propagation_.back().getLatestState()->getTransform());
  }
}

void RioFrontend::imuFilterCallback(const sensor_msgs::ImuConstPtr& msg) {
  LOG_FIRST(I, 1, "Received first filtered IMU message.");
  Eigen::Quaterniond q_IB;
  tf2::fromMsg(msg->orientation, q_IB);
  initial_state_ = std::make_shared<State>(
      initial_state_->odom_frame_id, initial_state_->I_p_IB, Rot3(q_IB),
      initial_state_->I_v_IB, msg, initial_state_->integrator);
}

void RioFrontend::cfarDetectionsCallback(
    const sensor_msgs::PointCloud2Ptr& msg) {
  LOG_FIRST(I, 1, "Received first CFAR detections.");
  if (propagation_.empty()) {
    LOG(W, "No propagation, skipping CFAR detections.");
    return;
  }

  Pose3 B_T_BR;
  try {
    auto R_T_RB_tf = tf_buffer_.lookupTransform(
        msg->header.frame_id,
        propagation_.back().getLatestState()->imu->header.frame_id,
        ros::Time(0), ros::Duration(0.0));
    Eigen::Affine3d R_T_RB_eigen = tf2::transformToEigen(R_T_RB_tf.transform);
    B_T_BR = Pose3(R_T_RB_eigen.inverse().matrix());
    LOG_FIRST(I, 1,
              "Looked up radar extrinsic calibration B_T_BR:\n"
                  << B_T_BR);
  } catch (tf2::TransformException& ex) {
    LOG(W, "Failed to lookup transform: " << ex.what()
                                          << ". Skipping CFAR detections.");
    return;
  }

  auto split_it = splitPropagation(msg->header.stamp);
  if (split_it == propagation_.end()) {
    LOG(W, "Failed to split propagation, skipping CFAR detections.");
    LOG(W, "Split time: " << msg->header.stamp);
    LOG(W, "Last IMU time: "
               << propagation_.back().getLatestState()->imu->header.stamp);
    return;
  }

  split_it->B_T_BR_ = B_T_BR;
  split_it->cfar_detections_ = parseRadarMsg(msg);
  // Track zero velocity detections.
  split_it->cfar_tracks_ =
      tracker_.addCfarDetections(split_it->cfar_detections_.value());
  optimization_.addRadarFactor(
      *split_it, *std::next(split_it), noise_model_radar_doppler_,
      noise_model_radar_track_, noise_model_radar_track_prior_);

  optimization_.solve(propagation_);
}

std::deque<Propagation>::iterator RioFrontend::splitPropagation(
    const ros::Time& t) {
  auto it = propagation_.begin();
  for (; it != propagation_.end(); ++it) {
    Propagation propagation_to_t, propagation_from_t;
    if (it->split(t, &idx_, &propagation_to_t, &propagation_from_t)) {
      *it = propagation_to_t;
      propagation_.insert(std::next(it), propagation_from_t);
      return it;
    }
  }

  return it;
}

std::vector<mav_sensors::Radar::CfarDetection> RioFrontend::parseRadarMsg(
    const sensor_msgs::PointCloud2Ptr& msg) const {
  std::vector<mav_sensors::Radar::CfarDetection> detections(msg->height *
                                                            msg->width);
  sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_doppler(*msg, "doppler");
  sensor_msgs::PointCloud2Iterator<int16_t> iter_snr(*msg, "snr");
  sensor_msgs::PointCloud2Iterator<int16_t> iter_noise(*msg, "noise");
  for (auto& detection : detections) {
    detection.x = *(iter_x);
    detection.y = *(iter_y);
    detection.z = *(iter_z);
    detection.velocity = *(iter_doppler);
    detection.snr = *(iter_snr);
    detection.noise = *(iter_noise);
    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_doppler;
    ++iter_snr;
    ++iter_noise;
  }
  return detections;
}