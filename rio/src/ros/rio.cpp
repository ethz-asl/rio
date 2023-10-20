#include "rio/ros/rio.h"

#include <cmath>

#include <geometry_msgs/Vector3Stamped.h>
#include <log++.h>
#include <nav_msgs/Odometry.h>
#include <tf2_eigen/tf2_eigen.h>

#include "rio/DopplerResidual.h"
#include "rio/Timing.h"
#include "rio/ros/common.h"

using namespace rio;
using namespace gtsam;

Rio::Rio(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {}

bool Rio::init() {
  // ROS communication.
  int queue_size = 1;
  if (!loadParam<int>(nh_private_, "queue_size", &queue_size)) return false;

  // Subscribers.
  imu_raw_sub_ =
      nh_.subscribe("imu/data_raw", queue_size, &Rio::imuRawCallback, this);
  imu_filter_sub_ =
      nh_.subscribe("imu/data", queue_size, &Rio::imuFilterCallback, this);
  radar_cfar_sub_ = nh_.subscribe("radar/cfar_detections", queue_size,
                                  &Rio::cfarDetectionsCallback, this);

  // Publishers
  odom_navigation_pub_ = nh_private_.advertise<nav_msgs::Odometry>(
      "odometry_navigation", queue_size);
  odom_optimizer_pub_ = nh_private_.advertise<nav_msgs::Odometry>(
      "odometry_optimizer", queue_size);
  timing_pub_ = nh_private_.advertise<rio::Timing>("timing", 100);
  acc_bias_pub_ =
      nh_private_.advertise<geometry_msgs::Vector3Stamped>("bias_acc", 100);
  gyro_bias_pub_ =
      nh_private_.advertise<geometry_msgs::Vector3Stamped>("bias_gyro", 100);
  doppler_residual_pub_ =
      nh_private_.advertise<rio::DopplerResidual>("doppler_residual", 100);

  // IMU integration
  PreintegratedCombinedMeasurements integrator;
  if (!loadPreintegratedCombinedMeasurements(nh_private_, &integrator))
    return false;

  // Initial state.
  std::string odom_frame_id = initial_state_->odom_frame_id;
  loadParam<std::string>(nh_private_, "odom_frame_id", &odom_frame_id);

  initial_state_ = std::make_shared<State>(
      odom_frame_id, initial_state_->I_p_IB, initial_state_->R_IB,
      initial_state_->I_v_IB, initial_state_->imu, integrator);

  // Prior noise pose.
  if (!loadPriorNoisePose(nh_private_, &prior_noise_model_I_T_IB_))
    return false;

  // Prior noise velocity.
  if (!loadPriorNoiseVelocity(nh_private_, &prior_noise_model_I_v_IB_))
    return false;

  // Prior noise IMU bias.
  if (!loadPriorNoiseImuBias(nh_private_, &prior_noise_model_imu_bias_))
    return false;

  // Noise Radar doppler.
  if (!loadNoiseRadarRadialVelocity(nh_private_, &noise_model_radar_doppler_))
    return false;

  // Noise Radar track.
  if (!loadNoiseRadarTrack(nh_private_, &noise_model_radar_track_))
    return false;

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

void Rio::imuRawCallback(const sensor_msgs::ImuConstPtr& msg) {
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
  auto new_odometry = propagation_.back().getLatestState()->getOdometry();
  odom_navigation_pub_.publish(new_odometry);

  if (new_result) {
    odom_optimizer_pub_.publish(new_odometry);

    tf_broadcaster_.sendTransform(
        propagation_.back().getLatestState()->getTransform());

    for (const auto& time : timing) timing_pub_.publish(time.second);

    geometry_msgs::Vector3Stamped bias_acc;
    tf2::toMsg(propagation_.back().getLatestState()->getBias().accelerometer(),
               bias_acc.vector);
    bias_acc.header = propagation_.back().getLatestState()->imu->header;
    acc_bias_pub_.publish(bias_acc);

    geometry_msgs::Vector3Stamped bias_gyro;
    tf2::toMsg(propagation_.back().getLatestState()->getBias().gyroscope(),
               bias_gyro.vector);
    bias_gyro.header = propagation_.back().getLatestState()->imu->header;
    gyro_bias_pub_.publish(bias_gyro);
  }
}

void Rio::imuFilterCallback(const sensor_msgs::ImuConstPtr& msg) {
  LOG_FIRST(I, 1, "Received first filtered IMU message.");
  Eigen::Quaterniond q_IB;
  tf2::fromMsg(msg->orientation, q_IB);
  initial_state_ = std::make_shared<State>(
      initial_state_->odom_frame_id, initial_state_->I_p_IB, Rot3(q_IB),
      initial_state_->I_v_IB, msg, initial_state_->integrator);
}

void Rio::cfarDetectionsCallback(const sensor_msgs::PointCloud2Ptr& msg) {
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
  std::vector<Vector1> doppler_residuals;
  optimization_.addRadarFactor(*split_it, *std::next(split_it),
                               noise_model_radar_doppler_,
                               noise_model_radar_track_, &doppler_residuals);

  optimization_.solve(propagation_);

  for (const auto& residual : doppler_residuals) {
    DopplerResidual residual_msg;
    residual_msg.header = msg->header;
    residual_msg.residual = residual[0];
    doppler_residual_pub_.publish(residual_msg);
  }
}

std::deque<Propagation>::iterator Rio::splitPropagation(const ros::Time& t) {
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