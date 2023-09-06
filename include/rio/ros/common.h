#include <vector>

#include <gtsam/base/Vector.h>
#include <log++.h>
#include <ros/ros.h>

namespace rio {
template <typename T>
bool loadParam(const ros::NodeHandle& nh, const std::string& name, T* value) {
  if (!nh.getParam(name, *value)) {
    LOG(F, "Failed to read " << nh.resolveName(name).c_str() << ".");
    return false;
  }
  return true;
}

template <>
bool loadParam<gtsam::Vector3>(const ros::NodeHandle& nh,
                               const std::string& name, gtsam::Vector3* value) {
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
bool loadParam<std::optional<gtsam::Vector3>>(
    const ros::NodeHandle& nh, const std::string& name,
    std::optional<gtsam::Vector3>* value) {
  std::optional<gtsam::Vector3> tmp = *value;
  tmp = {};
  if (!loadParam<gtsam::Vector3>(nh, name, &tmp.value())) return false;
  *value = tmp;
  return true;
}

}  // namespace rio