#include "rio/ros/radar.h"

#include <geometry_msgs/Vector3Stamped.h>
#include <log++.h>
#include <mav_sensors_core/sensor_config.h>
#include <sensor_msgs/PointCloud2.h>
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
      nh_private_.advertise<sensor_msgs::PointCloud2>("cfar_detections", 1);

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
  sensor_msgs::PointCloud2 msg;
  msg.header.stamp =
      rio::toRosTime(std::get<ms::Radar>(measurement).unix_stamp_ns);
  msg.header.frame_id = frame_id_;
  msg.height = 1;
  msg.width = std::get<ms::Radar>(measurement).cfar_detections.size();

  msg.fields.resize(6);
  msg.fields[0].name = "x";
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[0].count = 1;

  msg.fields[1].name = "y";
  msg.fields[1].offset = 4;
  msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[1].count = 1;

  msg.fields[2].name = "z";
  msg.fields[2].offset = 8;
  msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[2].count = 1;

  msg.fields[3].name = "doppler";
  msg.fields[3].offset = 12;
  msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[3].count = 1;

  msg.fields[4].name = "snr";
  msg.fields[4].offset = 16;
  msg.fields[4].datatype = sensor_msgs::PointField::INT16;
  msg.fields[4].count = 1;

  msg.fields[5].name = "noise";
  msg.fields[5].offset = 18;
  msg.fields[5].datatype = sensor_msgs::PointField::INT16;
  msg.fields[5].count = 1;

  int n = 1;
  msg.is_bigendian = *(char*)&n != 1;
  msg.point_step = 20;
  msg.row_step = msg.point_step * msg.width;
  msg.is_dense = true;

  msg.data.resize(msg.row_step * msg.height);
  for (size_t i = 0;
       i < std::get<ms::Radar>(measurement).cfar_detections.size(); i++) {
    char x[sizeof(float)];
    memcpy(x, &std::get<ms::Radar>(measurement).cfar_detections[i].x,
           sizeof(float));
    msg.data[i * msg.point_step + msg.fields[0].offset + 0] = x[0];
    msg.data[i * msg.point_step + msg.fields[0].offset + 1] = x[1];
    msg.data[i * msg.point_step + msg.fields[0].offset + 2] = x[2];
    msg.data[i * msg.point_step + msg.fields[0].offset + 3] = x[3];

    char y[sizeof(float)];
    memcpy(y, &std::get<ms::Radar>(measurement).cfar_detections[i].y,
           sizeof(float));
    msg.data[i * msg.point_step + msg.fields[1].offset + 0] = y[0];
    msg.data[i * msg.point_step + msg.fields[1].offset + 1] = y[1];
    msg.data[i * msg.point_step + msg.fields[1].offset + 2] = y[2];
    msg.data[i * msg.point_step + msg.fields[1].offset + 3] = y[3];

    char z[sizeof(float)];
    memcpy(z, &std::get<ms::Radar>(measurement).cfar_detections[i].z,
           sizeof(float));
    msg.data[i * msg.point_step + msg.fields[2].offset + 0] = z[0];
    msg.data[i * msg.point_step + msg.fields[2].offset + 1] = z[1];
    msg.data[i * msg.point_step + msg.fields[2].offset + 2] = z[2];
    msg.data[i * msg.point_step + msg.fields[2].offset + 3] = z[3];

    char doppler[sizeof(float)];
    memcpy(doppler,
           &std::get<ms::Radar>(measurement).cfar_detections[i].velocity,
           sizeof(float));
    msg.data[i * msg.point_step + msg.fields[3].offset + 0] = doppler[0];
    msg.data[i * msg.point_step + msg.fields[3].offset + 1] = doppler[1];
    msg.data[i * msg.point_step + msg.fields[3].offset + 2] = doppler[2];
    msg.data[i * msg.point_step + msg.fields[3].offset + 3] = doppler[3];

    char snr[sizeof(int16_t)];
    memcpy(snr, &std::get<ms::Radar>(measurement).cfar_detections[i].snr,
           sizeof(int16_t));
    msg.data[i * msg.point_step + msg.fields[4].offset + 0] = snr[0];
    msg.data[i * msg.point_step + msg.fields[4].offset + 1] = snr[1];

    char noise[sizeof(int16_t)];
    memcpy(noise, &std::get<ms::Radar>(measurement).cfar_detections[i].noise,
           sizeof(int16_t));
    msg.data[i * msg.point_step + msg.fields[5].offset + 0] = noise[0];
    msg.data[i * msg.point_step + msg.fields[5].offset + 1] = noise[1];
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