
#include <log++.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "rio/ros/radar.h"

namespace rio {

class RadarNodelet : public nodelet::Nodelet {
  virtual void onInit() {
    try {
      radar_ = std::make_shared<Radar>(getPrivateNodeHandle());
      radar_->init();
    } catch (std::runtime_error e) {
      LOG(E, "%s", e.what());
    }
  }

  std::shared_ptr<Radar> radar_;
};
}  // namespace rio

PLUGINLIB_EXPORT_CLASS(rio::RadarNodelet, nodelet::Nodelet)