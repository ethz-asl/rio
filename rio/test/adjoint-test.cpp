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

TEST(Adjoint, TransformTwistNoLinearVelocity) {
  auto omega_B = Vector3(0.0, 0.0, 1.0);
  auto v_B = Vector3(0.0, 0.0, 0.0);
  Vector6 V_B;
  V_B << omega_B, v_B;
  auto R_IB = Rot3();
  auto I_t_IB = Point3(2.0, 0.0, 0.0);
  auto I_T_IB = Pose3(R_IB, I_t_IB);

  auto V_I = I_T_IB.Adjoint(V_B);
  Vector6 V_I_expected;
  V_I_expected << 0.0, 0.0, 1.0, 0.0, -2.0, 0.0;
  EXPECT_TRUE(assert_equal(V_I_expected, V_I));
  EXPECT_TRUE(assert_equal(V_B, I_T_IB.inverse().Adjoint(V_I)));
}

TEST(Adjoint, TransformTwistWithLinearVelocity) {
  auto omega_B = Vector3(0.0, 0.0, 1.0);
  auto v_B = Vector3(1.0, 0.0, 0.0);
  Vector6 V_B;
  V_B << omega_B, v_B;
  auto R_IB = Rot3();
  auto I_t_IB = Point3(2.0, 0.0, 0.0);
  auto I_T_IB = Pose3(R_IB, I_t_IB);

  auto V_I = I_T_IB.Adjoint(V_B);
  Vector6 V_I_expected;
  V_I_expected << 0.0, 0.0, 1.0, 1.0, -2.0, 0.0;
  EXPECT_TRUE(assert_equal(V_I_expected, V_I));
  EXPECT_TRUE(assert_equal(V_B, I_T_IB.inverse().Adjoint(V_I)));
}

TEST(Adjoint, TransformWithRotation) {
  auto omega_B = Vector3(0.0, 0.0, 1.0);
  auto v_B = Vector3(1.0, 0.0, 0.0);
  Vector6 V_B;
  V_B << omega_B, v_B;
  auto R_IB = Rot3({0, -1, 0}, {1, 0, 0}, {0, 0, 1});
  auto I_t_IB = Point3(2.0, 0.0, 0.0);
  auto I_T_IB = Pose3(R_IB, I_t_IB);

  auto V_I = I_T_IB.Adjoint(V_B);
  Vector6 V_I_expected;
  V_I_expected << 0.0, 0.0, 1.0, 0.0, -3.0, 0.0;
  EXPECT_TRUE(assert_equal(V_I_expected, V_I));
  EXPECT_TRUE(assert_equal(V_B, I_T_IB.inverse().Adjoint(V_I)));
}

TEST(Adjoint, CalibrationExample) {
  auto R_BR = Rot3({0, -1, 0}, {1, 0, 0}, {0, 0, 1});
  auto B_t_BR = Point3(2.0, 0.0, 0.0);
  auto B_T_BR = Pose3(R_BR, B_t_BR);

  auto B_omega_IB = Vector3(0.0, 0.0, 1.0);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
