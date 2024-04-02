#pragma once

#include <vector>

#include <gtsam/base/Vector.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <log++.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace rio {
template <typename T>
inline bool loadParam(const ros::NodeHandle& nh, const std::string& name,
                      T* value) {
  if (!nh.getParam(name, *value)) {
    LOG(F, "Failed to read " << nh.resolveName(name).c_str() << ".");
    return false;
  }
  return true;
}

template <>
inline bool loadParam<gtsam::Vector3>(const ros::NodeHandle& nh,
                                      const std::string& name,
                                      gtsam::Vector3* value) {
  std::vector<double> vec;
  if (!loadParam<std::vector<double>>(nh, name, &vec)) return false;
  if (vec.size() != value->size()) {
    LOG(F, "Expected " << value->size() << " elements for " << name.c_str()
                       << ", got " << vec.size() << ".");
    return false;
  }
  *value = Eigen::Map<gtsam::Vector3>(vec.data(), vec.size());
  return true;
}

template <>
inline bool loadParam<gtsam::Vector4>(const ros::NodeHandle& nh,
                                      const std::string& name,
                                      gtsam::Vector4* value) {
  std::vector<double> vec;
  if (!loadParam<std::vector<double>>(nh, name, &vec)) return false;
  if (vec.size() != value->size()) {
    LOG(F, "Expected " << value->size() << " elements for " << name.c_str()
                       << ", got " << vec.size() << ".");
    return false;
  }
  *value = Eigen::Map<gtsam::Vector4>(vec.data(), vec.size());
  return true;
}

template <>
inline bool loadParam<std::optional<gtsam::Vector3>>(
    const ros::NodeHandle& nh, const std::string& name,
    std::optional<gtsam::Vector3>* value) {
  gtsam::Vector3 vec;
  if (!loadParam<gtsam::Vector3>(nh, name, &vec)) return false;
  *value = vec;
  return true;
}

bool loadPreintegratedCombinedMeasurements(
    const ros::NodeHandle& nh, gtsam::PreintegratedCombinedMeasurements* imu);

bool loadPriorNoisePose(const ros::NodeHandle& nh,
                        gtsam::SharedNoiseModel* noise);

bool loadPriorNoiseVelocity(const ros::NodeHandle& nh,
                            gtsam::SharedNoiseModel* noise);

bool loadPriorNoiseImuBias(const ros::NodeHandle& nh,
                           gtsam::SharedNoiseModel* noise);

bool loadNoiseRadarRadialVelocity(const ros::NodeHandle& nh,
                                  gtsam::SharedNoiseModel* noise);

bool loadNoiseRadarTrack(const ros::NodeHandle& nh,
                         gtsam::SharedNoiseModel* noise);

bool loadNoiseLoopClosureT(const ros::NodeHandle& nh,
                           gtsam::SharedNoiseModel* noise);

bool loadNoiseZeroVelocityPrior(const ros::NodeHandle& nh,
                                gtsam::SharedNoiseModel* noise);

bool loadNoiseBaroHeight(const ros::NodeHandle& nh,
                         gtsam::SharedNoiseModel* noise);

  struct CfarDetection {
    float x{std::nanf("1")};
    float y{std::nanf("1")};
    float z{std::nanf("1")};
    float velocity{std::nanf("1")};
    int16_t snr{-1};
    int16_t noise{-1};
    friend std::ostream& operator<<(std::ostream& os, const CfarDetection& cd) {
      os << "x: " << cd.x << " y: " << cd.y << " z: " << cd.z << " velocity: " << cd.velocity
         << " snr: " << cd.snr << " noise: " << cd.noise;
      return os;
    }
  };

std::vector<CfarDetection> parseRadarMsg(
    const sensor_msgs::PointCloud2Ptr& msg);

double computeBaroHeight(double pressure);

}  // namespace rio