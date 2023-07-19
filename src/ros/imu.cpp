#include "rio/ros/imu.h"

#include <log++.h>
#include <mav_sensors_core/sensor_config.h>
#include <sensor_msgs/Imu.h>

#include "rio/common.h"

using namespace rio;

Imu::Imu(const ros::NodeHandle& nh_private) : BaseSensor(nh_private) {}

Imu::~Imu() { imu_.close(); }

bool Imu::openSensor() {
  imu_pub_ = nh_private_.advertise<sensor_msgs::Imu>("imu_raw", 1);

  std::string path_acc;
  if (!nh_private_.getParam("path_acc", path_acc)) {
    LOG(F, "Failed to read IMU path_acc.");
    return false;
  }
  std::string path_gyro;
  if (!nh_private_.getParam("path_gyro", path_gyro)) {
    LOG(F, "Failed to read IMU path_gyro.");
    return false;
  }

  LOG(I, "Opening IMU on path_acc: " << path_acc.c_str()
                                     << " path_gyro: " << path_gyro.c_str());

  mav_sensors::SensorConfig cfg;
  cfg.set("path_acc", path_acc);
  cfg.set("path_gyro", path_gyro);
  imu_.setConfig(cfg);
  if (!imu_.open()) {
    LOG(F, "Failed to open IMU.");
    return false;
  }

  return true;
}

void Imu::readSensor() {
  // Read sensor data.
  auto measurement = imu_.read();

  if (std::get<0>(measurement).has_value() &&
      std::get<1>(measurement).has_value() &&
      std::get<2>(measurement).has_value()) {
    LOG_FIRST(I, 1, "Publishing first IMU measurement.");
    sensor_msgs::Imu msg;
    msg.header.stamp = rio::toRosTime(std::get<2>(measurement).value());
    msg.header.frame_id = frame_id_;
    msg.angular_velocity.x = std::get<1>(measurement).value().x;
    msg.angular_velocity.y = std::get<1>(measurement).value().y;
    msg.angular_velocity.z = std::get<1>(measurement).value().z;
    msg.linear_acceleration.x = std::get<0>(measurement).value().x;
    msg.linear_acceleration.y = std::get<0>(measurement).value().y;
    msg.linear_acceleration.z = std::get<0>(measurement).value().z;
    msg.orientation_covariance[0] = -1;  // orientation not supported by bmi088
    imu_pub_.publish(msg);
  }
}