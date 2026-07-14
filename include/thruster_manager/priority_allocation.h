#ifndef THRUSTER_MANAGER_PRIORITY_ALLOCATION_H
#define THRUSTER_MANAGER_PRIORITY_ALLOCATION_H

#include <Eigen/Core>
#include <algorithm>
#include <array>

namespace thruster_manager
{

/// Priority-tiered thrust allocation.
///
/// Decomposes the wrench into four tiers - heave; roll+pitch; yaw;
/// surge+sway - and allocates them in that order through the (linear)
/// pseudo-inverse, tracking cumulative per-thruster load. Each tier is scaled
/// UNIFORMLY by the largest fraction that fits in every thruster's remaining
/// headroom, so a saturated tier loses magnitude but never direction (no
/// parasitic cross-axis wrench from clipping), and lower tiers only consume
/// headroom left by higher ones: the vehicle gives up forward speed before it
/// gives up depth or attitude.
///
/// Pure Eigen, no ROS: unit-testable anywhere. When nothing saturates the
/// result equals tamInv * wrench exactly (the tier split is lossless through
/// a linear map).
///
/// fmin < 0 < fmax are the per-thruster limits (asymmetric supported - e.g.
/// T200 reverse thrust is weaker than forward). tier_scales reports the
/// applied fraction per tier (1 = unsaturated) so callers can log saturation.
inline Eigen::VectorXd allocatePrioritized(const Eigen::MatrixXd &tamInv,
                                           const Eigen::Matrix<double, 6, 1> &wrench,
                                           double fmin, double fmax,
                                           std::array<double, 4> &tier_scales)
{
  // axis indices per tier; -1 = unused slot
  static constexpr std::array<std::array<int, 2>, 4> TIERS{{
      {{2, -1}},  // heave
      {{3, 4}},   // roll + pitch
      {{5, -1}},  // yaw
      {{0, 1}},   // surge + sway
  }};

  const auto dofs{tamInv.rows()};
  Eigen::VectorXd total{Eigen::VectorXd::Zero(dofs)};
  tier_scales.fill(1.);

  for(size_t k = 0; k < TIERS.size(); ++k)
  {
    Eigen::Matrix<double, 6, 1> part;
    part.setZero();
    for(const auto axis: TIERS[k])
    {
      if(axis >= 0)
        part(axis) = wrench(axis);
    }
    if(part.isZero(0.))
      continue;

    const Eigen::VectorXd thrust{tamInv * part};

    double s{1.};
    for(Eigen::Index i = 0; i < dofs; ++i)
    {
      // headroom left on this thruster, sign-aware
      if(thrust(i) > 1e-9)
        s = std::min(s, (fmax - total(i)) / thrust(i));
      else if(thrust(i) < -1e-9)
        s = std::min(s, (fmin - total(i)) / thrust(i));
    }
    s = std::clamp(s, 0., 1.);
    tier_scales[k] = s;
    total += s * thrust;
  }
  return total;
}

} // namespace thruster_manager

#endif
