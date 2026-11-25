// Copyright 2026
// Licensed under the Apache License, Version 2.0.

#ifndef SMALL_GICP_RELOCALIZATION__REGISTRATION_QUALITY_HPP_
#define SMALL_GICP_RELOCALIZATION__REGISTRATION_QUALITY_HPP_

#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Geometry>

namespace small_gicp_relocalization
{
struct RegistrationQualityLimits
{
  double min_overlap{0.4};
  double max_rmse{0.2};
  double max_translation_delta{1.0};
  double max_rotation_delta{0.35};
};

struct RegistrationQuality
{
  const char * rejection{nullptr};
  double overlap{0.0};
  double rmse{std::numeric_limits<double>::infinity()};
  double translation_delta{0.0};
  double rotation_delta{0.0};
};

// Distances are physical nearest-neighbor squared distances, not GICP's
// covariance-weighted objective. Missing neighbors must remain infinity.
inline RegistrationQuality assessRegistration(
  bool converged, const Eigen::Isometry3d & candidate, const Eigen::Isometry3d & previous,
  const std::vector<double> & squared_distances, const RegistrationQualityLimits & limits)
{
  RegistrationQuality quality;
  if (!candidate.matrix().allFinite()) {
    quality.rejection = "non-finite transform";
    return quality;
  }
  const auto delta = previous.inverse() * candidate;
  quality.translation_delta = delta.translation().norm();
  quality.rotation_delta = std::abs(Eigen::AngleAxisd(delta.rotation()).angle());
  size_t supported = 0;
  double sum = 0.0;
  const double support_sq = 4.0 * limits.max_rmse * limits.max_rmse;
  for (const auto distance : squared_distances) {
    if (std::isfinite(distance) && distance >= 0.0 && distance <= support_sq) {
      ++supported;
      sum += distance;
    }
  }
  if (!squared_distances.empty()) {
    quality.overlap = static_cast<double>(supported) / squared_distances.size();
  }
  if (supported) {
    quality.rmse = std::sqrt(sum / supported);
  }
  if (!converged) {
    quality.rejection = "optimizer did not converge";
  } else if (quality.overlap < limits.min_overlap) {
    quality.rejection = "insufficient map overlap";
  } else if (quality.rmse > limits.max_rmse) {
    quality.rejection = "excessive matching RMSE";
  } else if (quality.translation_delta > limits.max_translation_delta ||
    quality.rotation_delta > limits.max_rotation_delta)
  {
    quality.rejection = "excessive map-to-odom correction jump";
  }
  return quality;
}
}  // namespace small_gicp_relocalization

#endif  // SMALL_GICP_RELOCALIZATION__REGISTRATION_QUALITY_HPP_
