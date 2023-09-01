#include "rio/gtsam/doppler_factor.h"

#include <log++.h>

using namespace rio;

DopplerFactor::DopplerFactor(
    gtsam::Key I_T_I_B_key, gtsam::Key I_v_IB_key, gtsam::Key bias_key,
    const gtsam::Vector& R_r_RT_measured, const double doppler_measured,
    const gtsam::Vector& B_omega_IB_measured, const gtsam::Pose3& B_T_BR,
    const gtsam::noiseModel::Base::shared_ptr& noise_model)
    : R_r_RT_measured_(R_r_RT_measured.normalized()),
      doppler_measured_(doppler_measured),
      B_omega_IB_measured_(B_omega_IB_measured),
      B_T_BR_(B_T_BR) {}

gtsam::Vector DopplerFactor::evaluateError(
    const gtsam::Pose3& I_T_I_B, const gtsam::Vector& I_v_IB,
    const gtsam::imuBias::ConstantBias& bias,
    boost::optional<gtsam::Matrix&> H_T, boost::optional<gtsam::Matrix&> H_v,
    boost::optional<gtsam::Matrix&> H_b) const {
  // e(T, v, b) = [R_BR^-1 (R_IB^-1 * I_v_IB + (B_omega_IB - bias_omega) x
  // B_r_BR)]^T * R_r_RT_measured / ||R_r_RT_measured|| - doppler_measured
  gtsam::Vector1 e;
  if (H_T) H_T->resize(3, 6);
  if (H_v) H_v->resize(3, 3);
  if (H_b) H_b->resize(3, 6);

  if (H_T || H_v || H_b) {
      // e = 
    I_T_I_B.rotation().unrotate(gtsam::Point3(I_v_IB), &H_T->leftCols<3>(), H_v) ;
  }

  return e;
}

void DopplerFactor::print(const std::string& text,
                          const gtsam::KeyFormatter& key_formatter) const {
  LOG(I, text << "DopplerFactor(" << key_formatter(this->key()) << ")");
  LOG(I, "Measured direction: " << R_r_RT_measured_.transpose());
  LOG(I, "Measured doppler: " << doppler_measured_);
  LOG(I, "Measured angular velocity: " << B_omega_IB_measured_.transpose());
  LOG(I, "Fixed extrinsic calibration:\n" << B_T_BR_);
  this->noiseModel_->print("Noise model: ");
}

bool DopplerFactor::equals(const gtsam::NonlinearFactor& expected,
                           double tolerance) const {
  const DopplerFactor::This* expected_casted =
      dynamic_cast<const DopplerFactor::This*>(&expected);  // NOLINT
  if (!expected_casted) return false;
  bool equal =
      gtsam::equal(R_r_RT_measured_, expected_casted->R_r_RT_measured_);
  equal &= gtsam::traits<double>::Equals(
      doppler_measured_, expected_casted->doppler_measured_, tolerance);
  equal &= gtsam::equal(B_omega_IB_measured_,
                        expected_casted->B_omega_IB_measured_, tolerance);
  equal &= (B_T_BR_.equals(expected_casted->B_T_BR_, tolerance));
  equal &= Base::equals(*expected_casted, tolerance);
  return equal;
}