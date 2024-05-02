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

  if (!loadParam<bool>(nh_private_, "baro/active", &baro_active_)) return false;
  if (baro_active_) {
    baro_sub_ =
        nh_.subscribe("baro/pressure", queue_size, &Rio::baroCallback, this);
  }

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
  baro_residual_pub_ =
      nh_private_.advertise<rio::DopplerResidual>("baro_residual", 100);

  // IMU integration
  PreintegratedCombinedMeasurements integrator;
  if (!loadPreintegratedCombinedMeasurements(nh_private_, &integrator))
    return false;

  // Initial state.
  std::string odom_frame_id = initial_state_->odom_frame_id;
  loadParam<std::string>(nh_private_, "odom_frame_id", &odom_frame_id);
  loadParam<int>(nh_private_, "imu/frequency", &imu_initialization_limit_);

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

  if (!loadNoiseBaroHeight(nh_private_, &noise_model_baro_height_))
    return false;

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

void Rio::baroCallback(const sensor_msgs::FluidPressureConstPtr& msg) {

	if(msg->fluid_pressure < 80000 || msg->fluid_pressure > 120000)
	{
		LOG(E, "Shit baro.");
		return;

	}
  LOG_FIRST(I, 1, "Received first barometer message.");
  baro_buffer_.push_back(
      std::make_pair(msg->header.stamp.toSec(), msg->fluid_pressure));
  while (!baro_buffer_.empty() &&
         baro_buffer_.front().first < optimization_.cutoff_time) {
    baro_buffer_.pop_front();
  }
}

void Rio::imuRawCallback(const sensor_msgs::ImuConstPtr& msg) {
  LOG_FIRST(I, 1, "Received first raw IMU message.");
  // Initialize.
  if (initial_state_->imu == nullptr) {
     Eigen::Vector3d shit_acc_data;
     shit_acc_data << msg->linear_acceleration.x, msg->linear_acceleration.y,
                msg->linear_acceleration.z;
     if(shit_acc_data.norm() < 9.5){
     	LOG(E, "Shit Accelerometer");
	return;
     }

    LOG_TIMED(W, 1.0, "Initial state not complete, acummulating measurements.");
    accumulated_imu_ +=
        Vector3(msg->linear_acceleration.x, msg->linear_acceleration.y,
                msg->linear_acceleration.z);
    accumulated_imu_count_++;
    if (accumulated_imu_count_ == imu_initialization_limit_) {
      Vector3 b_a =
          accumulated_imu_ / static_cast<double>(imu_initialization_limit_) -
          Vector3(0., 0., 9.81);
      auto params = std::make_shared<PreintegrationCombinedParams>(
          initial_state_->integrator.p());
      auto b_g = initial_state_->integrator.biasHat().gyroscope();
      auto preint_correct_biases =
          PreintegratedCombinedMeasurements(params, {b_a, b_g});
      LOG(I, "Corrected IMU Biases:\nAccel:\t"
                 << b_a[0] << " , " << b_a[1] << " , " << b_a[2] << "\nGyro:\t"
                 << b_a[0] << " , " << b_g[1] << " , " << b_g[2]);

      // assume we are always level with the ground to set acceleration
      // biases
      Eigen::Quaterniond q_IB = Eigen::Quaterniond::Identity();
      //      double roll = atan2(accumulated_imu_.y(), accumulated_imu_.z());
      //      double pitch = atan2(-accumulated_imu_.x(),
      //                           sqrt(accumulated_imu_.y() *
      //                           accumulated_imu_.y() +
      //                                accumulated_imu_.z() *
      //                                accumulated_imu_.z()));
      //      Eigen::Quaterniond q_IB =
      //          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      //          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
      initial_state_ = std::make_shared<State>(
          initial_state_->odom_frame_id, initial_state_->I_p_IB, Rot3(q_IB),
          initial_state_->I_v_IB, msg, preint_correct_biases);
    }
    return;
  } else if (optimization_.smoother_failed_.load()) {
    initial_state_ = std::make_shared<State>(linked_propagations_.head->state_);
    linked_propagations_.remove(std::numeric_limits<double>::infinity());
    optimization_.smoother_failed_.store(false);
  }
  if (!linked_propagations_.head) {
    LOG(I, "Initializing states with initial state.");
    linked_propagations_.head = new Propagation(*initial_state_, idx_++);
    optimization_.addPriorFactor(
        *linked_propagations_.head, prior_noise_model_I_T_IB_,
        prior_noise_model_I_v_IB_, prior_noise_model_imu_bias_);

    auto newhead = new Propagation(*initial_state_, idx_++);
    newhead->prior = linked_propagations_.head;
    linked_propagations_.head = newhead;
    linked_propagations_.head->imu_measurements_.push_back(msg);
    return;
  }

  // Get update from optimization.
  auto new_result = optimization_.getResult(linked_propagations_);

  // Integrate.
  if (!linked_propagations_.head->addImuMeasurement(msg)) {
    LOG(W, "Failed to add IMU measurement, skipping IMU integration.");
    return;
  }
  // Publish.
  auto new_odometry = linked_propagations_.head->state_.getOdometry();
  odom_navigation_pub_.publish(new_odometry);

  if (new_result) {
    // cant start imuRawCallback timer above because threaded solving which also
    // is timed with gttic...
    gttic_(imuRawCallback);
    odom_optimizer_pub_.publish(new_odometry);

    tf_broadcaster_.sendTransform(
        linked_propagations_.head->state_.getTransform());

    for (const auto& time : optimization_.timing_)
      timing_pub_.publish(time.second);

    geometry_msgs::Vector3Stamped bias_acc;
    tf2::toMsg(linked_propagations_.head->state_.getBias().accelerometer(),
               bias_acc.vector);

    bias_acc.header =
        linked_propagations_.head->imu_measurements_.back()->header;
    acc_bias_pub_.publish(bias_acc);

    geometry_msgs::Vector3Stamped bias_gyro;
    tf2::toMsg(linked_propagations_.head->state_.getBias().gyroscope(),
               bias_gyro.vector);
    bias_gyro.header =
        linked_propagations_.head->imu_measurements_.back()->header;
    gyro_bias_pub_.publish(bias_gyro);
    gttoc_(imuRawCallback);
    tictoc_finishedIteration_();
    tictoc_getNode(imuRawCallback, imuRawCallback);
    if (optimization_.timing_.find("optimize") != optimization_.timing_.end())
      optimization_.updateTiming(
          imuRawCallback, "imuRawCallback",
          optimization_.timing_["optimize"].header.stamp);
  }
}

void Rio::imuFilterCallback(const sensor_msgs::ImuConstPtr& msg) {
  LOG_FIRST(I, 1, "Madgwick filter on, use to set initial orientation.");
  if (!initial_state_->imu) {
    Eigen::Quaterniond q_IB;
    tf2::fromMsg(msg->orientation, q_IB);
    initial_state_ = std::make_shared<State>(
        initial_state_->odom_frame_id, initial_state_->I_p_IB, Rot3(q_IB),
        initial_state_->I_v_IB, msg, initial_state_->integrator);
  }
}

void Rio::cfarDetectionsCallback(const sensor_msgs::PointCloud2Ptr& msg) {
  gttic_(cfarDetectionsCallback);
  if (msg->header.stamp.toSec() == 0) {
    LOG(W, "Received CFAR detections with t=0, skipping...");
    return;
  }
  if (initial_state_->imu == nullptr) {
    return;
  }
  Pose3 B_T_BR;
  try {
    auto R_T_RB_tf = tf_buffer_.lookupTransform(
        msg->header.frame_id,
        linked_propagations_.head->imu_measurements_.back()->header.frame_id,
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
  auto propagation_split =
      linked_propagations_.getSplitPropagation(msg->header.stamp);
  if (!propagation_split) {
    LOG(W, "Failed to split propagation, skipping CFAR detections. t_msg: "
               << msg->header.stamp.toSec());
    return;
  }

  Propagation* new_propagation = new Propagation();
  {
    std::scoped_lock lock(optimization_.values_mutex_);
    if (optimization_.optimized_values_.size() > 0) {
      propagation_split->prior->updateState(optimization_.optimized_values_);
    }
  }

  new_propagation->state_ = propagation_split->prior->state_;

  if (!linked_propagations_.insertPrior(new_propagation, propagation_split,
                                        msg->header.stamp, idx_)) {
    LOG(W, "Failed to insert prior, skipping CFAR detections.");
    return;
  }

  new_propagation->B_T_BR_ = B_T_BR;
  new_propagation->cfar_detections_ = parseRadarMsg(msg);
  // Track zero velocity detections.
  new_propagation->cfar_tracks_ =
      tracker_.addCfarDetections(new_propagation->cfar_detections_.value());
  std::vector<Vector1> doppler_residuals;
  optimization_.addRadarFactor(*new_propagation, *propagation_split,
                               noise_model_radar_doppler_,
                               noise_model_radar_track_, &doppler_residuals);

  if (new_propagation->graph_idx_ < 30) {
    optimization_.addPriorFactor(*new_propagation, *initial_state_,
                                 prior_noise_model_I_T_IB_,
                                 prior_noise_model_I_v_IB_);
  }

  Vector1 baro_residual;
  if (baro_active_ && !baro_buffer_.empty()) {
    if (baro_height_offset_ == 0.0) {
      double baro_sum = std::accumulate(
          baro_buffer_.begin(), baro_buffer_.end(), 0.0,
          [](double accumulator, const std::pair<double, double>& entry) {
            return accumulator + entry.second;
          });
      baro_height_offset_ = computeBaroHeight(
          baro_sum / static_cast<double>(baro_buffer_.size()));
      LOG(E, "Baro height offset: " << baro_height_offset_);
    }

    // get latest baro measurement which is smaller than the radar measurement
    // start from back
    auto baro_it = std::lower_bound(
        baro_buffer_.rbegin(), baro_buffer_.rend(),
        std::make_pair(msg->header.stamp.toSec(), 0.0),
        [](const std::pair<double, double>& a,
           const std::pair<double, double>& b) { return a.first > b.first; });

    if (baro_it == baro_buffer_.rend()) {
      baro_it = std::prev(baro_it);
    }

    if (std::prev(baro_it) != baro_buffer_.rbegin() - 1) {
      auto baro_it_prev = std::prev(baro_it);
      if (baro_it_prev->first - msg->header.stamp.toSec() <
          msg->header.stamp.toSec() - baro_it->first) {
        baro_it = baro_it_prev;
      }
    } else if (baro_it->first <
               new_propagation->prior->state_.imu->header.stamp.toSec()) {
      LOG(W, "No close baro measurement found, skipping baro factor.");
      baro_it == baro_buffer_.rend();
    }
    if (baro_it != baro_buffer_.rend()) {
      auto baro_height = computeBaroHeight(baro_it->second);
      optimization_.addBaroFactor(*new_propagation, baro_height,
                                  baro_height_offset_, noise_model_baro_height_,
                                  &baro_residual);
      DopplerResidual baro_residual_msg;
      baro_residual_msg.header = msg->header;
      baro_residual_msg.residual = baro_residual[0];
      baro_residual_pub_.publish(baro_residual_msg);
    } else {
      LOG(W, "Failed to find baro measurement with stamp before or at "
                 << msg->header.stamp << ". Skipping baro factor.");
    }
  }

  gttoc_(cfarDetectionsCallback);
  tictoc_finishedIteration_();
  tictoc_getNode(cfarDetectionsCallback, cfarDetectionsCallback);
  if (optimization_.timing_.find("optimize") != optimization_.timing_.end())
    optimization_.updateTiming(cfarDetectionsCallback, "cfarDetectionsCallback",
                               optimization_.timing_["optimize"].header.stamp);

  optimization_.solve(linked_propagations_);

  for (const auto& residual : doppler_residuals) {
    DopplerResidual residual_msg;
    residual_msg.header = msg->header;
    residual_msg.residual = residual[0];
    doppler_residual_pub_.publish(residual_msg);
  }
}
