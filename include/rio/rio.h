/*
BSD 3-Clause License

Copyright (c) 2024 ETH Zurich, Autonomous Systems Lab, Rik Girod

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include <deque>
#include <optional>
#include <vector>

#include <gtsam/linear/NoiseModel.h>
#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "rio/gtsam/landmark_tracker.h"
#include "rio/gtsam/optimization.h"
#include "rio/gtsam/propagation.h"
#include "rio/gtsam/state.h"

// This class implements a callback driven sensor fusion.
// IMU raw callback: preintegrate IMU measurements and publish the result.
// IMU filter callback: get initial orientation from external filter.
// radar measurement callback: split preintegration to add radar factor, start
// optimization.
namespace rio {
class Rio {
 public:
  Rio(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  bool init();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber imu_raw_sub_;
  ros::Subscriber imu_filter_sub_;
  ros::Subscriber radar_cfar_sub_;
  ros::Subscriber baro_sub_;
  void imuRawCallback(const sensor_msgs::ImuConstPtr& msg);
  void imuFilterCallback(const sensor_msgs::ImuConstPtr& msg);
  void cfarDetectionsCallback(const sensor_msgs::PointCloud2Ptr& msg);
  void pressureCallback(const sensor_msgs::FluidPressurePtr& msg);
  void processIMUMeasurements(const sensor_msgs::ImuConstPtr& msg);
  void processRadarFrames();

  ros::Publisher odom_navigation_pub_;
  ros::Publisher odom_optimizer_pub_;
  ros::Publisher timing_pub_;
  ros::Publisher acc_bias_pub_;
  ros::Publisher gyro_bias_pub_;
  ros::Publisher doppler_residual_pub_;
  ros::Publisher baro_residual_pub_;

  State::ConstPtr initial_state_{std::make_shared<State>(
      "odom", gtsam::Z_3x1, gtsam::Rot3(), gtsam::Z_3x1, nullptr,
      gtsam::PreintegratedCombinedMeasurements())};
  std::deque<Propagation> propagation_;
  ros::Duration max_dead_reckoning_duration_{60.0};

  std::deque<Propagation>::iterator splitPropagation(const ros::Time& t);

  Optimization optimization_;
  gtsam::SharedNoiseModel prior_noise_model_I_T_IB_;
  gtsam::SharedNoiseModel prior_noise_model_I_v_IB_;
  gtsam::SharedNoiseModel prior_noise_model_imu_bias_;
  gtsam::SharedNoiseModel noise_model_radar_doppler_;
  gtsam::SharedNoiseModel noise_model_radar_track_;
  gtsam::SharedNoiseModel noise_model_baro_height_;
  uint64_t idx_{0};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_{tf_buffer_};
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  Tracker tracker_;

  bool baro_active_{false};
  std::optional<double> baro_height_bias_;
  std::deque<std::pair<double, double>> baro_height_bias_history_;

  std::deque<sensor_msgs::ImuConstPtr> imu_queue;
  std::deque<sensor_msgs::PointCloud2Ptr> radar_queue;

  std::string dataset_ = "rio";
};
}  // namespace rio