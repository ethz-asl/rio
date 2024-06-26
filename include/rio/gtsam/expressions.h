/*
BSD 3-Clause License

Copyright (c) 2024 ETH Zurich, Autonomous Systems Lab, Rik Girod

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

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

inline gtsam::Double_ norm3_(const gtsam::Vector3_& v) {
  double (*f)(const gtsam::Vector3&, gtsam::OptionalJacobian<1, 3>) =
      &gtsam::norm3;
  return gtsam::Double_(f, v);
}

inline gtsam::Unit3_ FromPoint3_(const gtsam::Point3_& point3) {
  gtsam::Unit3 (*f)(const gtsam::Point3&, gtsam::OptionalJacobian<2, 3>) =
      &gtsam::Unit3::FromPoint3;
  return gtsam::Unit3_(f, point3);
}

inline gtsam::Double_ dot(const gtsam::Unit3_& a, const gtsam::Unit3_& b) {
  return gtsam::Double_(a, &gtsam::Unit3::dot, b);
}

inline gtsam::Double_ radialVelocity_(const gtsam::Vector3_& R_v_IR,
                                      const gtsam::Unit3_& R_p_TR_unit) {
  return norm3_(R_v_IR) * dot(FromPoint3_(R_v_IR), R_p_TR_unit);
}
}  // namespace rio