// Priority-tiered allocation unit tests (pure Eigen, no ROS).
#include <gtest/gtest.h>
#include "thruster_manager/priority_allocation.h"
#include <Eigen/Dense>
#include <cmath>

using thruster_manager::allocatePrioritized;
using Vector6d = Eigen::Matrix<double, 6, 1>;

namespace
{

// 8-thruster TAM: 4 verticals + 4 x 45-degree vectored horizontals
Eigen::MatrixXd makeTam()
{
  const double c = std::sqrt(0.5);
  Eigen::MatrixXd dirs(3, 8), pos(3, 8);
  dirs << c, 0, 0, -c, -c, 0, 0, c,
         -c, 0, 0, -c,  c, 0, 0, c,
          0, 1, 1,  0,  0, 1, 1, 0;
  pos << 0.29926, 0.12700, -0.12700, -0.29926, -0.29926, -0.12700, 0.12700, 0.29926,
         0.20874, 0.23950,  0.23950,  0.20874, -0.20874, -0.23950, -0.23950, -0.20874,
        -0.10340, -0.02052, -0.02052, -0.10340, -0.10340, -0.02052, -0.02052, -0.10340;
  Eigen::MatrixXd tam(6, 8);
  for(int i = 0; i < 8; ++i)
  {
    const Eigen::Vector3d d{dirs.col(i)}, p{pos.col(i)};
    tam.col(i).head<3>() = d;
    tam.col(i).tail<3>() = p.cross(d);
  }
  return tam;
}

constexpr double FMAX = 51.6, FMIN = -40.0;

} // namespace

TEST(PriorityAllocation, UnsaturatedEqualsPlainPinv)
{
  const auto tam{makeTam()};
  const Eigen::MatrixXd tamInv{tam.completeOrthogonalDecomposition().pseudoInverse()};
  std::array<double, 4> s{};

  Vector6d w; w << 5, 3, 8, 0.5, 0.5, 1.0;
  const Eigen::VectorXd tiered{allocatePrioritized(tamInv, w, FMIN, FMAX, s)};
  EXPECT_LT((tiered - tamInv * w).norm(), 1e-9);
  EXPECT_LT((tam * tiered - w).norm(), 1e-9);
  for(double v: s)
    EXPECT_GT(v, 0.999);
}

TEST(PriorityAllocation, SurgeSaturationPreservesHeaveAndYaw)
{
  const auto tam{makeTam()};
  const Eigen::MatrixXd tamInv{tam.completeOrthogonalDecomposition().pseudoInverse()};
  std::array<double, 4> s{};

  Vector6d big; big << 500, 0, 30, 0, 0, 3;
  const Eigen::VectorXd thrust{allocatePrioritized(tamInv, big, FMIN, FMAX, s)};
  const Vector6d got{tam * thrust};
  EXPECT_NEAR(got(2), 30.0, 1e-9);  // heave fully served
  EXPECT_NEAR(got(5), 3.0, 1e-9);   // yaw fully served
  EXPECT_GT(s[0], 0.999);
  EXPECT_LT(s[3], 1.0);             // surge attenuated
  EXPECT_GT(got(0), 0.0);
  EXPECT_LT(got(0), 500.0);
  EXPECT_NEAR(got(1), 0.0, 1e-9);   // no parasitic sway
  for(int i = 0; i < 8; ++i)
  {
    EXPECT_LE(thrust(i), FMAX + 1e-9);
    EXPECT_GE(thrust(i), FMIN - 1e-9);
  }
}

TEST(PriorityAllocation, AsymmetricLimitsRespected)
{
  const auto tam{makeTam()};
  const Eigen::MatrixXd tamInv{tam.completeOrthogonalDecomposition().pseudoInverse()};
  std::array<double, 4> s{};

  Vector6d down; down << 0, 0, -300, 0, 0, 0;
  const Eigen::VectorXd thrust{allocatePrioritized(tamInv, down, FMIN, FMAX, s)};
  EXPECT_LT(s[0], 1.0);
  EXPECT_NEAR(thrust.minCoeff(), FMIN, 1e-6);  // pinned at the reverse limit
}

TEST(PriorityAllocation, ZeroWrenchZeroThrust)
{
  const auto tam{makeTam()};
  const Eigen::MatrixXd tamInv{tam.completeOrthogonalDecomposition().pseudoInverse()};
  std::array<double, 4> s{};
  EXPECT_LT(allocatePrioritized(tamInv, Vector6d::Zero(), FMIN, FMAX, s).norm(), 1e-12);
}
