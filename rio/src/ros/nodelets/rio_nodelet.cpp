#include <memory>

#include <log++.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "rio/ros/rio.h"

namespace rio {

class RioNodelet : public nodelet::Nodelet {
  virtual void onInit() {
    try {
      rio_ = std::make_shared<Rio>(getNodeHandle(), getPrivateNodeHandle());
      rio_->init();
    } catch (std::runtime_error e) {
      LOG(E, "%s", e.what());
    }
  }

  std::shared_ptr<Rio> rio_;
};
}  // namespace rio

PLUGINLIB_EXPORT_CLASS(rio::RioNodelet, nodelet::Nodelet)