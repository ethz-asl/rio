#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/expressions.h>

#include "rio/gtsam/functions.h"

namespace rio {

typedef gtsam::Expression<gtsam::imuBias::ConstantBias> ConstantBias_;

// Note: Input vector from target to radar (negative target location).
inline gtsam::Double_ radialVelocity_(const gtsam::Vector3_& R_v_IR,
                                      const gtsam::Point3_& R_p_TR) {
  return dot(R_v_IR, gtsam::normalize(R_p_TR));
}

inline gtsam::Vector3_ correctGyroscope_(const ConstantBias_& bias,
                                         const gtsam::Vector3_& measurement) {
  return gtsam::Vector3_(bias, &gtsam::imuBias::ConstantBias::correctGyroscope,
                         measurement);
}
}  // namespace rio