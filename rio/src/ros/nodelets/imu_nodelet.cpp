#include <memory>

#include <log++.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "rio/ros/imu.h"

namespace rio {

class ImuNodelet : public nodelet::Nodelet {
  virtual void onInit() {
    try {
      imu_ = std::make_unique<Imu>(getPrivateNodeHandle());
      if (!imu_->init()) imu_.release();
    } catch (std::runtime_error e) {
      LOG(E, "%s", e.what());
    }
  }

  std::unique_ptr<Imu> imu_;
};
}  // namespace rio

PLUGINLIB_EXPORT_CLASS(rio::ImuNodelet, nodelet::Nodelet)