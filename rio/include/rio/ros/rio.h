#pragma once

#include <atomic>
#include <mutex>
#include <optional>
#include <thread>

#include <gtsam/geometry/Rot3.h>
#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

// This class implements a callback driven sensor fusion.
// IMU raw callback: preintegrate IMU measurements and publish the result.
// IMU filter callback: get initial orientation from external filter.
// radar trigger callback: create new key.
// radar measurement callback: add radar measurement to the current key, start
// optimization thread.
namespace rio {
class Rio {
 public:
  Rio(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  ~Rio();
  bool init();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber imu_raw_sub_;
  ros::Subscriber imu_filter_sub_;
  ros::Subscriber radar_trigger_sub_;
  ros::Subscriber radar_cfar_sub_;
  void imuRawCallback(const sensor_msgs::ImuConstPtr& msg);
  void imuFilterCallback(const sensor_msgs::ImuConstPtr& msg);
  void radarTriggerCallback(const std_msgs::HeaderConstPtr& msg);
  void cfarDetectionsCallback(const sensor_msgs::PointCloud2& msg);

  ros::Publisher odom_integrated_pub_;
  ros::Publisher odom_optimized_pub_;

  struct State {
    gtsam::Rot3 R_IB;
    gtsam::Vector3 I_v_IB;
    gtsam::Point3 I_r_IB;
    gtsam::Vector3 bias_gyro;
    gtsam::Vector3 bias_accel;
  };
};
}  // namespace rio