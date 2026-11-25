// Copyright 2026 tcp
// SPDX-License-Identifier: Apache-2.0

#ifndef SMALL_GICP_RELOCALIZATION__REGISTERED_CLOUD_WINDOW_HPP_
#define SMALL_GICP_RELOCALIZATION__REGISTERED_CLOUD_WINDOW_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace small_gicp_relocalization
{

// Clouds must already be motion compensated into the same fixed odom frame.
class RegisteredCloudWindow
{
public:
  using Cloud = pcl::PointCloud<pcl::PointXYZ>;
  struct Frame
  {
    int64_t stamp_ns;
    Cloud::ConstPtr cloud;
  };
  struct Update
  {
    bool accepted;
    bool time_reset;
  };

  RegisteredCloudWindow(int64_t duration_ns, size_t max_points)
  : duration_ns_(duration_ns), max_points_(max_points) {}

  Update push(int64_t stamp_ns, const Cloud::ConstPtr & cloud)
  {
    if (stamp_ns < 0 || cloud->empty() || cloud->size() > max_points_) {
      return {false, false};
    }
    const bool time_reset = have_stamp_ && stamp_ns < last_stamp_ns_;
    if (time_reset) {
      frames_.clear();
      point_count_ = 0;
    } else if (have_stamp_ && stamp_ns == last_stamp_ns_) {
      return {false, false};  // Do not give duplicate messages extra weight.
    }
    pruneBefore(stamp_ns - duration_ns_);
    while (!frames_.empty() && point_count_ + cloud->size() > max_points_) {
      popFront();
    }
    frames_.push_back({stamp_ns, cloud});
    point_count_ += cloud->size();
    last_stamp_ns_ = stamp_ns;
    have_stamp_ = true;
    return {true, time_reset};
  }

  void pruneBefore(int64_t cutoff_ns)
  {
    while (!frames_.empty() && frames_.front().stamp_ns < cutoff_ns) {
      popFront();
    }
  }

  std::vector<Frame> snapshot() const {return {frames_.begin(), frames_.end()};}
  size_t pointCount() const {return point_count_;}

private:
  void popFront()
  {
    point_count_ -= frames_.front().cloud->size();
    frames_.pop_front();
  }

  int64_t duration_ns_;
  size_t max_points_;
  std::deque<Frame> frames_;
  size_t point_count_{0};
  int64_t last_stamp_ns_{0};
  bool have_stamp_{false};
};

}  // namespace small_gicp_relocalization

#endif  // SMALL_GICP_RELOCALIZATION__REGISTERED_CLOUD_WINDOW_HPP_
