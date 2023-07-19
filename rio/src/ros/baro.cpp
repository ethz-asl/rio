#include "rio/ros/baro.h"

#include <log++.h>
#include <mav_sensors_core/sensor_config.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Temperature.h>

#include "rio/common.h"

using namespace rio;

Baro::Baro(const ros::NodeHandle& nh_private) : BaseSensor(nh_private) {}

Baro::~Baro() { baro_.close(); }

bool Baro::openSensor() {
  baro_pub_ = nh_private_.advertise<sensor_msgs::FluidPressure>("pressure", 1);
  temp_pub_ = nh_private_.advertise<sensor_msgs::Temperature>("temperature", 1);

  std::string path;
  if (!nh_private_.getParam("path", path)) {
    LOG(F, "Failed to read barometer path.");
    return false;
  } else {
    LOG(I, "Opening barometer on path: " << path.c_str());
  }

  SensorConfig cfg;
  cfg.set("path", path);
  baro_.setConfig(cfg);
  if (!baro_.open()) {
    LOG(F, "Failed to open barometer.");
    return false;
  }

  return true;
}

void Baro::readSensor() {
  // Read sensor data.
  auto measurement = baro_.read();

  if (std::get<0>(measurement).has_value() && std::get<2>(measurement).has_value()) {
    // Publish pressure measurements.
    LOG_FIRST(I, 1, "Publishing first pressure measurement.");
    sensor_msgs::FluidPressure msg;
    msg.header.stamp = rio::toRosTime(std::get<2>(measurement).value());
    msg.header.frame_id = frame_id_;
    msg.fluid_pressure = std::get<0>(measurement).value();
    baro_pub_.publish(msg);
  }
  if (std::get<1>(measurement).has_value() && std::get<2>(measurement).has_value()) {
    // Publish temperature measurements.
    LOG_FIRST(I, 1, "Publishing first temperature measurement.");
    sensor_msgs::Temperature msg;
    msg.header.stamp = rio::toRosTime(std::get<2>(measurement).value());
    msg.header.frame_id = frame_id_;
    msg.temperature = std::get<1>(measurement).value();
    temp_pub_.publish(msg);
  }
}