#include "rio/gtsam/doppler_factor.h"

#include <gtest/gtest.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>

using namespace rio;
using namespace gtsam;
using symbol_shorthand::B;
using symbol_shorthand::V;
using symbol_shorthand::X;

TEST(DopplerFactor, Jacobian) {
  Rot3 R = Rot3::Rodrigues(0.1, 0.2, 0.3);
  Point3 t(1.0, 0.5, 0.2);
  Pose3 I_T_IB(R, t);
  imuBias::ConstantBias bias{Vector3(0.1, 0.2, 0.3), Vector3(0.1, -0.2, 0.1)};
  Vector3 I_v_IB{0.1, 0.2, 0.3};

  Vector3 R_r_RT_measured{10.0, 1.0, 1.0};
  double doppler_measured{1.0};
  Vector3 B_omega_IB_measured{0.0, 0.0, 1.0};
  Pose3 B_T_BR({-0.09229596, 0.70105738, 0.70105738, -0.09229596},
               {0.115, -0.002, -0.018});

  DopplerFactor factor(X(1), V(1), B(1), R_r_RT_measured, doppler_measured,
                       B_omega_IB_measured, B_T_BR,
                       noiseModel::Isotropic::Sigma(1, 0.10));

  Matrix actual_H_T, actual_H_v, actual_H_b;
  factor.evaluateError(I_T_IB, I_v_IB, bias, actual_H_T, actual_H_v,
                       actual_H_b);
  Matrix numerical_H_T =
      numericalDerivative31<Vector, Pose3, Vector3, imuBias::ConstantBias>(
          std::bind(&DopplerFactor::evaluateError, &factor,
                    std::placeholders::_1, std::placeholders::_2,
                    std::placeholders::_3, boost::none, boost::none,
                    boost::none),
          I_T_IB, I_v_IB, bias);

  Matrix numerical_H_v =
      numericalDerivative32<Vector, Pose3, Vector3, imuBias::ConstantBias>(
          std::bind(&DopplerFactor::evaluateError, &factor,
                    std::placeholders::_1, std::placeholders::_2,
                    std::placeholders::_3, boost::none, boost::none,
                    boost::none),
          I_T_IB, I_v_IB, bias);

  Matrix numerical_H_b =
      numericalDerivative33<Vector, Pose3, Vector3, imuBias::ConstantBias>(
          std::bind(&DopplerFactor::evaluateError, &factor,
                    std::placeholders::_1, std::placeholders::_2,
                    std::placeholders::_3, boost::none, boost::none,
                    boost::none),
          I_T_IB, I_v_IB, bias);

  EXPECT_TRUE(assert_equal(numerical_H_T, actual_H_T, 1e-8));
  EXPECT_TRUE(assert_equal(numerical_H_v, actual_H_v, 1e-8));
  EXPECT_TRUE(assert_equal(numerical_H_b, actual_H_b, 1e-8));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
