#pragma once

#include <mav_sensors_drivers/radar/xwr18xx_mmw_demo.h>
#include <ros/ros.h>

#include "rio/ros/base_sensor.h"

namespace rio {
class Radar : public BaseSensor {
 private:
  void readSensor() override;
  bool openSensor() override;

  mav_sensors::Xwr18XxMmwDemo radar_;
  ros::Publisher ls_vel_pub_;
  ros::Publisher cfar_pub_;

 public:
  Radar(const ros::NodeHandle& nh_private);
  ~Radar();
};
}  // namespace rio