#pragma once

#include <mav_sensors_drivers/barometer/bmp390.h>
#include <ros/ros.h>

#include "rio/ros/base_sensor.h"

namespace rio {
class Baro : public BaseSensor {
 private:
  void readSensor() override;
  bool openSensor() override;

  BMP390<Spi> baro_;
  ros::Publisher baro_pub_;
  ros::Publisher temp_pub_;

 public:
  Baro(const ros::NodeHandle& nh_private);
  ~Baro();
};
}  // namespace rio