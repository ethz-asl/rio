
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <log++.h>

#include "rio/ros/baro.h"

namespace rio {

class BaroNodelet : public nodelet::Nodelet {
  virtual void onInit() {
    try {
      baro_ = std::make_shared<Baro>(getPrivateNodeHandle());
      baro_->init();
    } catch (std::runtime_error e) {
      LOG(E, "%s", e.what());
    }
  }

  std::shared_ptr<Baro> baro_;
};
}  // namespace rio

PLUGINLIB_EXPORT_CLASS(rio::BaroNodelet, nodelet::Nodelet)