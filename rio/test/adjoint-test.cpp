#include <random>

#include <gtest/gtest.h>
#include <gtsam/geometry/Pose3.h>

using namespace gtsam;
std::mt19937 engine(42);

TEST(Adjoint, Create) {
  auto R_IB = Rot3::Random(engine);
  auto t_IB = Point3(1.0, 2.0, 3.0);
  auto I_T_IB = Pose3(R_IB, t_IB);

  Matrix66 Ad_IB;
  Ad_IB << R_IB.matrix(), Z_3x3,
      skewSymmetric(t_IB.x(), t_IB.y(), t_IB.z()) * R_IB.matrix(),
      R_IB.matrix();
  
  EXPECT_TRUE(assert_equal(Ad_IB, I_T_IB.AdjointMap()));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
