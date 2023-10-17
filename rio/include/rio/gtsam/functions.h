#pragma once

#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/geometry/Unit3.h>

namespace rio {

double radialVelocity(const gtsam::Vector3& R_v_IR,
                      const gtsam::Point3& R_p_RT,
                      gtsam::OptionalJacobian<1, 3> H_v = {},
                      gtsam::OptionalJacobian<1, 3> H_p = {});

}