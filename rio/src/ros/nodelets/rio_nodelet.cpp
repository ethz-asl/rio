#include <memory>

#include <log++.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "rio/ros/rio.h"

namespace rio {

class RioNodelet : public nodelet::Nodelet {
  virtual void onInit() {
    try {
      rio_ = std::make_unique<Rio>(getNodeHandle(), getPrivateNodeHandle());
      if (!rio_->init()) rio_.release();
    } catch (std::runtime_error e) {
      LOG(E, "%s", e.what());
    }
  }

  std::unique_ptr<Rio> rio_;
};
}  // namespace rio

PLUGINLIB_EXPORT_CLASS(rio::RioNodelet, nodelet::Nodelet)