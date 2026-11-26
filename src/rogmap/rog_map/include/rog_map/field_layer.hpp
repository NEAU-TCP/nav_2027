#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <mutex>
#include <vector>

namespace rog_map {

enum class InterpolationMode
{
  BILINEAR = 0,
  QUADRATIC = 1
};

struct FieldBuildStats
{
  double total_time_ms{0.0};
  double edt_positive_time_ms{0.0};
  double inverse_mask_time_ms{0.0};
  double edt_negative_time_ms{0.0};
  double distance_fill_time_ms{0.0};
  double commit_time_ms{0.0};
};

class DynamicLayer
{
public:
  DynamicLayer() = default;

  void update(int width,
    int height,
    double resolution,
    const Eigen::Vector2d & origin,
    const std::vector<uint8_t> & mask,
    double inflation_radius,
    double max_distance = 3.0,
    double min_distance = -1.0,
    bool clamp_distance = true,
    InterpolationMode interpolation = InterpolationMode::BILINEAR,
    FieldBuildStats * stats = nullptr);

  void rebuild(int width,
    int height,
    double resolution,
    const Eigen::Vector2d & origin,
    const std::vector<uint8_t> & mask,
    double inflation_radius,
    double max_distance = 3.0,
    double min_distance = -1.0,
    bool clamp_distance = true,
    InterpolationMode interpolation = InterpolationMode::BILINEAR,
    FieldBuildStats * stats = nullptr);

  // Rebuild only the region influenced by changed projection columns. The
  // region is expanded by max_distance so values near its edge still see all
  // possible obstacle sources. Geometry and the existing distance field must
  // already be initialized.
  void updateDirty(int width,
    int height,
    double resolution,
    const Eigen::Vector2d & origin,
    const std::vector<uint8_t> & mask,
    const std::vector<int> & dirty_columns,
    double inflation_radius,
    double max_distance = 3.0,
    double min_distance = -1.0,
    bool clamp_distance = true,
    InterpolationMode interpolation = InterpolationMode::BILINEAR,
    FieldBuildStats * stats = nullptr);

  void evaluate(const Eigen::Vector3d & pos, double & dist, Eigen::Vector3d & grad) const;

  bool isValid() const;
  bool matchesGeometry(int width, int height, double resolution, const Eigen::Vector2d & origin) const;
  int width() const;
  int height() const;
  double resolution() const;
  Eigen::Vector2d origin() const;
  std::vector<double> distances() const;
  double maxDistance() const;
  double minDistance() const;
  bool clampDistanceEnabled() const;
  InterpolationMode interpolationMode() const;

private:
  static constexpr double kFarDistance = 10.0;

  mutable std::mutex mutex_;
  std::vector<double> dist_m_;
  int width_{0};
  int height_{0};
  double resolution_{0.0};
  Eigen::Vector2d origin_{0.0, 0.0};
  double max_distance_{3.0};
  double min_distance_{-1.0};
  bool clamp_distance_{true};
  InterpolationMode interpolation_{InterpolationMode::BILINEAR};
};

}  // namespace rog_map
