#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/expressions.h>

#include "rio/gtsam/functions.h"

namespace rio {
// Note: Input vector from target to radar (negative target location).
inline gtsam::Double_ radialVelocity_(const gtsam::Vector3_& R_v_IR,
                                      const gtsam::Point3_& R_p_TR) {
  return dot(R_v_IR, gtsam::normalize(R_p_TR));
}
}  // namespace rio