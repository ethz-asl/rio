#include "rio/ros/radar.h"

#include <geometry_msgs/Vector3Stamped.h>
#include <log++.h>
#include <mav_sensors_core/sensor_config.h>
#include <sensor_msgs/PointCloud.h>
#include <tf2_eigen/tf2_eigen.h>

#include "rio/common.h"
#include "rio/least_squares.h"

using namespace rio;

namespace ms = mav_sensors;

Radar::Radar(const ros::NodeHandle& nh_private) : BaseSensor(nh_private) {}

Radar::~Radar() { radar_.close(); }

bool Radar::openSensor() {
  ls_vel_pub_ =
      nh_private_.advertise<geometry_msgs::Vector3Stamped>("ls_velocity", 1);
  cfar_pub_ =
      nh_private_.advertise<sensor_msgs::PointCloud>("cfar_detections", 1);

  std::string path_cfg;
  if (!nh_private_.getParam("path_cfg", path_cfg)) {
    LOG(F, "Failed to read radar path_cfg.");
    return false;
  }
  std::string path_data;
  if (!nh_private_.getParam("path_data", path_data)) {
    LOG(F, "Failed to read radar path_data.");
    return false;
  }

  std::string radar_cfg;
  if (!nh_private_.getParam("radar_cfg", radar_cfg)) {
    LOG(F, "Failed to read radar radar_cfg.");
    return false;
  }

  std::string trigger;
  if (!nh_private_.getParam("trigger", trigger)) {
    LOG(F, "Failed to read radar trigger.");
    return false;
  }

  LOG(I, "Opening radar on path_cfg: " << path_cfg.c_str()
                                       << " path_data: " << path_data.c_str());

  mav_sensors::SensorConfig cfg;
  cfg.set("path_cfg_file", radar_cfg);
  cfg.set("path_cfg", path_cfg);
  cfg.set("path_data", path_data);
  cfg.set("trigger", trigger);

  if (trigger == "true") {
    std::string trigger_delay;
    if (!nh_private_.getParam("trigger_delay", trigger_delay)) {
      LOG(F, "Failed to read radar trigger_delay.");
      return false;
    }

    std::string trigger_gpio;
    if (!nh_private_.getParam("trigger_gpio", trigger_gpio)) {
      LOG(F, "Failed to read radar trigger_gpio.");
      return false;
    }

    std::string trigger_gpio_name;
    if (!nh_private_.getParam("trigger_gpio_name", trigger_gpio_name)) {
      LOG(F, "Failed to read radar trigger_gpio_name.");
      return false;
    }

    cfg.set("trigger_delay", trigger_delay);
    cfg.set("trigger_gpio", trigger_gpio);
    cfg.set("trigger_gpio_name", trigger_gpio_name);
  }

  radar_.setConfig(cfg);
  if (!radar_.open()) {
    LOG(F, "Failed to open radar.");
    return false;
  }

  return true;
}

void Radar::readSensor() {
  // Read sensor data.
  auto measurement = radar_.read();

  // Publish radar detections as PointCloud messages.
  LOG_FIRST(I, 1, "Publishing first radar cfar detections.");
  sensor_msgs::PointCloud msg;
  msg.header.stamp =
      rio::toRosTime(std::get<ms::Radar>(measurement).unix_stamp_ns);
  msg.header.frame_id = "awr1843aop";
  msg.points.resize(std::get<ms::Radar>(measurement).cfar_detections.size());
  msg.channels.resize(3);
  msg.channels[0].name = "velocity";
  msg.channels[0].values.resize(
      std::get<ms::Radar>(measurement).cfar_detections.size());
  msg.channels[1].name = "snr";
  msg.channels[1].values.resize(
      std::get<ms::Radar>(measurement).cfar_detections.size());
  msg.channels[2].name = "noise";
  msg.channels[2].values.resize(
      std::get<ms::Radar>(measurement).cfar_detections.size());
  for (size_t i = 0;
       i < std::get<ms::Radar>(measurement).cfar_detections.size(); i++) {
    msg.points[i].x = std::get<ms::Radar>(measurement).cfar_detections[i].x;
    msg.points[i].y = std::get<ms::Radar>(measurement).cfar_detections[i].y;
    msg.points[i].z = std::get<ms::Radar>(measurement).cfar_detections[i].z;
    msg.channels[0].values[i] =
        std::get<ms::Radar>(measurement).cfar_detections[i].velocity;
    msg.channels[1].values[i] =
        std::get<ms::Radar>(measurement).cfar_detections[i].snr;
    msg.channels[2].values[i] =
        std::get<ms::Radar>(measurement).cfar_detections[i].noise;
  }
  cfar_pub_.publish(msg);

  // RANSAC least squares fit to estimate linear velocity.
  Eigen::Vector3d velocity;
  if (rio::leastSquares(std::get<ms::Radar>(measurement), &velocity)) {
    LOG_FIRST(I, 1, "Publishing first radar least squares velocity estimate.");
    // Publish least squares velocity estimate.
    geometry_msgs::Vector3Stamped msg_velocity;
    msg_velocity.header.stamp =
        rio::toRosTime(std::get<ms::Radar>(measurement).unix_stamp_ns);
    msg_velocity.header.frame_id = frame_id_;
    tf2::toMsg(velocity, msg_velocity.vector);
    ls_vel_pub_.publish(msg_velocity);
  } else {
    LOG(D, "Least squares failed.");
  }
}