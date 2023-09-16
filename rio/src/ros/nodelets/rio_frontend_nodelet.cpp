#include <memory>

#include <log++.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "rio/ros/rio_frontend.h"

namespace rio {

class RioFrontendNodelet : public nodelet::Nodelet {
  virtual void onInit() {
    try {
      rio_frontend_ = std::make_unique<RioFrontend>(getNodeHandle(),
                                                    getPrivateNodeHandle());
      if (!rio_frontend_->init()) rio_frontend_.release();
    } catch (std::runtime_error e) {
      LOG(E, "%s", e.what());
    }
  }

  std::unique_ptr<RioFrontend> rio_frontend_;
};
}  // namespace rio

PLUGINLIB_EXPORT_CLASS(rio::RioFrontendNodelet, nodelet::Nodelet)