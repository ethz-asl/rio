#include "rio/gtsam/doppler_factor.h"

#include <log++.h>

using namespace rio;

DopplerFactor::DopplerFactor(
    gtsam::Key I_T_I_B_key, gtsam::Key I_v_IB_key, gtsam::Key bias_key,
    const gtsam::Vector3& R_r_RT_measured, const double doppler_measured,
    const gtsam::Vector3& B_omega_IB_measured, const gtsam::Pose3& B_T_BR,
    const gtsam::noiseModel::Base::shared_ptr& noise_model,
    const double min_distance)
    : Base(noise_model, I_T_I_B_key, I_v_IB_key, bias_key),
      R_r_RT_measured_(R_r_RT_measured.normalized()),
      doppler_measured_(doppler_measured),
      B_omega_IB_measured_(B_omega_IB_measured),
      B_T_BR_(B_T_BR),
      min_distance_(min_distance) {}

gtsam::Vector DopplerFactor::evaluateError(
    const gtsam::Pose3& I_T_IB, const gtsam::Vector3& I_v_IB,
    const gtsam::imuBias::ConstantBias& bias, gtsam::OptionalMatrixType H_T,
    gtsam::OptionalMatrixType H_v, gtsam::OptionalMatrixType H_b) const {
  // e(T, v, b) = [R_BR^-1 (R_IB^-1 * I_v_IB + (B_omega_IB - bias_omega) x
  // B_r_BR)]^T * R_r_RT_measured / ||R_r_RT_measured|| + doppler_measured
  // Expanded:
  // e(T, v, b) = I_v_IB^T * R_IB * R_BR * R_r_RT_measured / ||R_r_RT_measured||
  // + B_r_BR^T [(B_omega_IB - bias_omega)]x^T * R_BR * R_r_RT_measured /
  // ||R_r_RT_measured||
  // + doppler_measured

  if (H_T) {
    H_T->resize(1, 6);
    H_T->setZero();
  }
  if (H_v) {
    H_v->resize(1, 3);
    H_v->setZero();
  }
  if (H_b) {
    H_b->resize(1, 6);
    H_b->setZero();
  }

  double distance = R_r_RT_measured_.norm();
  if (distance < min_distance_) {
    LOG(E, "DopplerFactor: Radar point is too close to radar. Distance: "
               << distance << "m");
    return gtsam::Vector1(0.0);
  }

  gtsam::Vector3 radar_proj_body =
      B_T_BR_.rotation().rotate(R_r_RT_measured_ / distance);
  gtsam::Vector3 radar_proj_body_leverarm =
      radar_proj_body.transpose() *
      gtsam::skewSymmetric(B_T_BR_.translation().x(), B_T_BR_.translation().y(),
                           B_T_BR_.translation().z())
          .transpose();

  gtsam::Matrix H_R_IB_raw, H_I_v_IB_raw, H_b_raw;
  H_R_IB_raw.resize(3, 3);
  H_I_v_IB_raw.resize(3, 3);
  H_b_raw.resize(3, 6);
  double e_v = radar_proj_body.dot(
      I_T_IB.rotation().unrotate(I_v_IB, &H_R_IB_raw, &H_I_v_IB_raw));
  double e_omega = radar_proj_body_leverarm.dot(
      bias.correctGyroscope(B_omega_IB_measured_, &H_b_raw));

  if (H_T) H_T->leftCols<3>() = radar_proj_body.transpose() * H_R_IB_raw;

  if (H_v) *H_v = radar_proj_body.transpose() * H_I_v_IB_raw;

  if (H_b) *H_b = radar_proj_body_leverarm.transpose() * H_b_raw;

  return gtsam::Vector1(e_v + e_omega + doppler_measured_);
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