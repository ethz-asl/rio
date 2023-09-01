#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace rio {
// Radar doppler velocity and direction factor.
class DopplerFactor
    : public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Vector,
                                      gtsam::imuBias::ConstantBias> {
 public:
  DopplerFactor(gtsam::Key I_T_I_B_key, gtsam::Key I_v_IB_key,
                gtsam::Key bias_key, const gtsam::Vector& R_r_RT_measured,
                const double doppler_measured,
                const gtsam::Vector& B_omega_IB_measured,
                const gtsam::Pose3& B_T_BR,
                const gtsam::noiseModel::Base::shared_ptr& noise_model);

  gtsam::Vector evaluateError(
      const gtsam::Pose3& I_T_I_B, const gtsam::Vector& I_v_IB,
      const gtsam::imuBias::ConstantBias& bias,
      boost::optional<gtsam::Matrix&> H_T = boost::none,
      boost::optional<gtsam::Matrix&> H_v = boost::none,
      boost::optional<gtsam::Matrix&> H_b = boost::none) const override;

  void print(const std::string& s,
             const gtsam::KeyFormatter& key_formatter =
                 gtsam::DefaultKeyFormatter) const override;

  inline gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::NonlinearFactor::shared_ptr(new This(*this));
  }

  bool equals(const gtsam::NonlinearFactor& expected,
              double tolerance) const override;

 protected:
  typedef gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Vector,
                                   gtsam::imuBias::ConstantBias>
      Base;
  typedef DopplerFactor This;

 private:
  gtsam::Vector R_r_RT_measured_;
  double doppler_measured_;
  gtsam::Vector B_omega_IB_measured_;
  gtsam::Pose3 B_T_BR_;
};

// Radar doppler velocity and direction factor. Additionally estimates the
// transform from IMU to radar frame.
}  // namespace rio