#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <log++.h>

namespace rio {
// Radar doppler velocity and direction factor.
class DopplerFactor1
    : public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Vector,
                                      gtsam::imuBias::ConstantBias> {
 public:
  DopplerFactor1(gtsam::Key I_T_I_B_key, gtsam::Key I_v_IB_key,
                 gtsam::Key bias_key, const gtsam::Vector& R_r_RT_measured,
                 const double doppler_measured,
                 const gtsam::Vector& B_omega_IB_measured,
                 const gtsam::Pose3& B_T_BR,
                 const gtsam::noiseModel::Base::shared_ptr& noise_model);

  gtsam::Vector unwhitenedError(const gtsam::Values& x,
                                boost::optional<std::vector<gtsam::Matrix>&> H =
                                    boost::none) const override;

  void print(const std::string& s,
             const gtsam::KeyFormatter& key_formatter =
                 gtsam::DefaultKeyFormatter) const override;

 private:
  gtsam::Vector R_r_RT_measured_;
  double doppler_measured_;
  gtsam::Vector B_omega_IB_measured_;
  gtsam::Pose3 B_T_BR_;
};

// Radar doppler velocity and direction factor. Additionally estimates the
// transform from IMU to radar frame.
}  // namespace rio