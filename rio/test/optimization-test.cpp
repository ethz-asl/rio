#include <gtest/gtest.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include "rio/gtsam/doppler_factor.h"

using namespace rio;
using namespace gtsam;
using symbol_shorthand::B;
using symbol_shorthand::V;
using symbol_shorthand::X;

TEST(Optimization, calculateEstimate) {
  auto noise_model_radar =
      gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(1) << 0.1).finished());

  gtsam::IncrementalFixedLagSmoother smoother;
  gtsam::NonlinearFactorGraph graph;
  uint64_t idx = 342;
  graph.add(DopplerFactor(
      X(idx), V(idx), B(idx), {-0.375, 0.926498, 0.03125}, 0,
      {0.840298, 1.47859, 0.510523},
      gtsam::Pose3({-1.10409e-08, 0.965926, -0.258819, 1, -1.10409e-08,
                    2.95841e-09, 2.95841e-09, -0.258819, -0.965926},
                   {0.115, -0.002, -0.018}),
      noise_model_radar, 0.1));
  graph.add(DopplerFactor(
      X(idx), V(idx), B(idx), {-0.125 ,0.991664 , 0.03125}, 0,
      {0.840298  1.47859 0.510523},
      gtsam::Pose3({-1.10409e-08, 0.965926, -0.258819, 1, -1.10409e-08,
                    2.95841e-09, 2.95841e-09, -0.258819, -0.965926},
                   {0.115, -0.002, -0.018}),
      noise_model_radar, 0.1));


}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
