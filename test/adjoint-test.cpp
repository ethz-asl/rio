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

#include <random>

#include <gtest/gtest.h>
#include <gtsam/geometry/Pose3.h>

using namespace gtsam;
std::mt19937 engine(42);

TEST(Adjoint, Create) {
  auto R_IB = Rot3::Random(engine);
  auto I_t_IB = Point3(1.0, 2.0, 3.0);
  auto I_T_IB = Pose3(R_IB, I_t_IB);

  Matrix66 Ad_IB;
  Ad_IB << R_IB.matrix(), Z_3x3,
      skewSymmetric(I_t_IB.x(), I_t_IB.y(), I_t_IB.z()) * R_IB.matrix(),
      R_IB.matrix();

  EXPECT_TRUE(assert_equal(Ad_IB, I_T_IB.AdjointMap()));
}

TEST(Adjoint, CalibrationExample) {
  // Given state variables.
  auto R_IB = Rot3({0, -1, 0}, {1, 0, 0}, {0, 0, 1});
  auto I_t_IB = Point3(4.0, 0.0, 0.0);
  auto I_T_IB = Pose3(R_IB, I_t_IB);

  auto R_BR = Rot3({0, -1, 0}, {1, 0, 0}, {0, 0, 1});
  auto B_t_BR = Point3(2.0, 0.0, 0.0);
  auto B_T_BR = Pose3(R_BR, B_t_BR);

  auto B_omega_IB = Vector3(0.0, 0.0, 1.0);
  auto I_v_IB = Vector3(1.0, 0.0, 0.0);

  // Goal: velocity in radar frame.
  auto B_v_IB = I_T_IB.rotation().unrotate(I_v_IB);
  Vector6 V_B = (Vector6() << B_omega_IB, B_v_IB).finished();
  Vector6 V_B_expected = (Vector6() << 0.0, 0.0, 1.0, 0.0, 1.0, 0.0).finished();
  EXPECT_TRUE(assert_equal(V_B_expected, V_B));

  auto I_v_IR_expected = Vector3(3, 0, 0);
  auto I_omega_IR_expected = Vector3(0, 0, 1);
  auto R_v_IR_expected = (I_T_IB * B_T_BR).rotation().unrotate(I_v_IR_expected);
  auto R_omega_IR_expected =
      (I_T_IB * B_T_BR).rotation().unrotate(I_omega_IR_expected);
  auto V_R_expected =
      (Vector6() << R_omega_IR_expected, R_v_IR_expected).finished();

  auto V_R = B_T_BR.inverse().Adjoint(V_B);
  EXPECT_TRUE(assert_equal(V_R_expected, V_R));

  // Oneliner. Twist in body frame, then transform to radar frame.
  auto V_R_oneliner = B_T_BR.inverse().Adjoint(
      (Vector6() << B_omega_IB, I_T_IB.rotation().unrotate(I_v_IB)).finished());
  EXPECT_TRUE(assert_equal(V_R_oneliner, V_R));
  Vector3 R_v_IR_oneliner = V_R_oneliner.tail<3>();
  EXPECT_TRUE(assert_equal(R_v_IR_expected, R_v_IR_oneliner));

  // Oneliner, but this time with cross product instead of adjoint (we are not
  // interested in angular velocity).
  // I_v_IR = I_v_IB + I_omega_IB x I_t_BR
  // R_v_IR = R_RI * I_v_IR = R_RI * (I_v_IB + R_IB * (B_omega_IB x B_t_BR))
  auto R_v_IR_oneliner_cross =
      (I_T_IB * B_T_BR)
          .rotation()
          .unrotate(I_v_IB + I_T_IB.rotation().rotate(
                                 B_omega_IB.cross(B_T_BR.translation())));
  EXPECT_TRUE(assert_equal(R_v_IR_expected, R_v_IR_oneliner_cross));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
