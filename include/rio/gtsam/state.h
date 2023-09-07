#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <nav_msgs/Odometry.h>
#include <ros/time.h>

namespace rio {
struct State {
  std::optional<ros::Time> stamp;
  std::optional<std::string> odom_frame_id;
  std::optional<std::string> body_frame_id;

  // State variables.
  std::optional<gtsam::Point3> I_p_IB;
  std::optional<gtsam::Rot3> R_IB;
  std::optional<gtsam::Vector3> I_v_IB;
  std::optional<gtsam::Vector3> b_a;
  std::optional<gtsam::Vector3> b_g;

  // Measurements.
  std::optional<gtsam::Vector3> B_omega_IB;

  bool hasFullState() const;
  nav_msgs::Odometry getOdometry() const;
  geometry_msgs::TransformStamped getTransform() const;
  geometry_msgs::Vector3Stamped getBiasAcc() const;
  geometry_msgs::Vector3Stamped getBiasGyro() const;
  gtsam::NavState getNavState() const;
  gtsam::imuBias::ConstantBias getBias() const;
};
}  // namespace rio