#include "rio/gtsam/doppler_factor.h"

using namespace rio;

DopplerFactor1::DopplerFactor1(
    gtsam::Key I_T_I_B_key, gtsam::Key I_v_IB_key, gtsam::Key bias_key,
    const gtsam::Vector& R_r_RT_measured, const double doppler_measured,
    const gtsam::Vector& B_omega_IB_measured, const gtsam::Pose3& B_T_BR,
    const gtsam::noiseModel::Base::shared_ptr& noise_model)
    : R_r_RT_measured_(R_r_RT_measured.normalized()),
      doppler_measured_(doppler_measured),
      B_omega_IB_measured_(B_omega_IB_measured),
      B_T_BR_(B_T_BR) {}