#pragma once

#include <deque>

#include <mav_sensors_drivers/imu/bmi088.h>
#include <mav_sensors_core/common/vec.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include "rio/ros/base_sensor.h"

namespace rio {
class Imu : public BaseSensor {
 private:
  void readSensor() override;
  bool openSensor() override;

  mav_sensors::Bmi088<mav_sensors::Spi> imu_;
  ros::Publisher imu_pub_, bias_pub_;
  ros::ServiceServer calibrate_srv_;
  std::deque<mav_sensors::vec3<double>> omega_;
  mav_sensors::vec3<double> b_g_ {0.0, 0.0, 0.0};
  int bias_samples_ = 0;

 public:
  Imu(const ros::NodeHandle& nh_private);
  ~Imu();

  bool calibrate(std_srvs::Trigger::Request& req,
                 std_srvs::Trigger::Response& res);
};
}  // namespace rio