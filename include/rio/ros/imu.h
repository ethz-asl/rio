#pragma once

#include <mav_sensors_drivers/imu/bmi088.h>
#include <ros/ros.h>

#include "rio/ros/base_sensor.h"

namespace rio {
class Imu : public BaseSensor {
 private:
  void readSensor() override;
  bool openSensor() override;

  Bmi088<Spi> imu_;
  ros::Publisher imu_pub_;

 public:
  Imu(const ros::NodeHandle& nh_private);
  ~Imu();
};
}  // namespace rio