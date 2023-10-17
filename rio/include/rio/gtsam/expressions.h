#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/expressions.h>

#include "rio/gtsam/functions.h"

namespace rio {
inline gtsam::Double_ radialVelocity_(const gtsam::Vector3_& R_v_IR,
                                      const gtsam::Point3_& R_p_RT) {
  return gtsam::Double_(&radialVelocity, R_v_IR, R_p_RT);
}
}  // namespace rio