
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "rio/ros/baro.h"

namespace rio {

class BaroNodelet : public nodelet::Nodelet {
  virtual void onInit() {
    try {
      baro_ = std::make_shared<Baro>(getPrivateNodeHandle());
    } catch (std::runtime_error e) {
      ROS_ERROR("%s", e.what());
    }
  }

  std::shared_ptr<Baro> baro_;
};
}  // namespace rio

PLUGINLIB_EXPORT_CLASS(rio::BaroNodelet, nodelet::Nodelet)