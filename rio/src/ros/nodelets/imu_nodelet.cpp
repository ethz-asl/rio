
#include <log++.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "rio/ros/imu.h"

namespace rio {

class ImuNodelet : public nodelet::Nodelet {
  virtual void onInit() {
    try {
      imu_ = std::make_shared<Imu>(getPrivateNodeHandle());
      imu_->init();
    } catch (std::runtime_error e) {
      LOG(E, "%s", e.what());
    }
  }

  std::shared_ptr<Imu> imu_;
};
}  // namespace rio

PLUGINLIB_EXPORT_CLASS(rio::ImuNodelet, nodelet::Nodelet)