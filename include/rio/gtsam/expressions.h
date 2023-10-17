#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/expressions.h>

namespace rio {
inline gtsam::Double_ radialVelocity_(const gtsam::Vector3& R_v_IR,
                                      const gtsam::Point3& R_p_RT) {
  return Double_(&radialVelocity, R_v_IR, R_p_RT);
}
}  // namespace rio