#include "rio/gtsam/functions.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Unit3.h>

using namespace rio;
using namespace gtsam;

double radialVelocity(const Vector3& R_v_IR, const Point3& R_p_RT,
                      OptionalJacobian<1, 3> H_v = {},
                      OptionalJacobian<1, 3> H_p = {}) {
  // Get the bearing vector.
  Matrix23 H_b_p;
  const Unit3 b = Unit3::FromPoint3(R_p_RT, H_p ? &H_b_p : nullptr);

  // Convert bearing vector to 3D point.
  Matrix32 H_pn_p;
  const Point3 pn = b.point3(H_p ? &H_pn_p : nullptr);

  // Compute the dot product.
  Matrix13 H_dot_v, H_dot_pn;
  const double d =
      dot(R_v_IR, pn, H_v ? &H_dot_v : nullptr, H_p ? &H_dot_pn : nullptr);

  if (H_v) {
    (*H_v) << H_dot_v;
  }
  if (H_p) {
    (*H_p) << H_dot_pn * H_pn_p * H_b_p;
  }

  return d;
}