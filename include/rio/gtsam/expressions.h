#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/expressions.h>

namespace rio {

typedef gtsam::Expression<gtsam::imuBias::ConstantBias> ConstantBias_;

inline gtsam::Vector3_ correctGyroscope_(const ConstantBias_& bias,
                                         const gtsam::Vector3_& measurement) {
  return gtsam::Vector3_(bias, &gtsam::imuBias::ConstantBias::correctGyroscope,
                         measurement);
}

inline gtsam::Double_ norm3_(const gtsam::Vector3_& v){
  double (*f)(const gtsam::Vector3 &, gtsam::OptionalJacobian<1, 3>) = &gtsam::norm3;
  return gtsam::Double_(f, v);
}

inline gtsam::Unit3_ FromPoint3_(const gtsam::Point3_& point3){
  gtsam::Unit3 (*f)(const gtsam::Point3 &, gtsam::OptionalJacobian<2, 3>) = &gtsam::Unit3::FromPoint3;
  return gtsam::Unit3_(f, point3);
}

inline gtsam::Double_ dot(const gtsam::Unit3_& a,
                          const gtsam::Unit3_& b) {
  return gtsam::Double_(a, &gtsam::Unit3::dot, b);
}

inline gtsam::Double_ radialVelocity_(const gtsam::Vector3_& R_v_IR,
                                      const gtsam::Unit3_& R_p_TR_unit) {                            
  return norm3_(R_v_IR) * dot(FromPoint3_(R_v_IR), R_p_TR_unit);
}
}  // namespace rio