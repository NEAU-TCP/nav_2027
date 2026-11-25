// Copyright 2026 tcp
// SPDX-License-Identifier: Apache-2.0

#include "small_gicp_relocalization/registered_cloud_window.hpp"

#include "gtest/gtest.h"

namespace small_gicp_relocalization
{
namespace
{
RegisteredCloudWindow::Cloud::Ptr cloud(size_t count, float x = 0.0F)
{
  RegisteredCloudWindow::Cloud::Ptr points(new RegisteredCloudWindow::Cloud());
  points->resize(count);
  for (auto & point : *points) {
    point.x = x;
    point.y = 0.0F;
    point.z = 0.0F;
  }
  return points;
}

TEST(RegisteredCloudWindow, StartupDoesNotWaitForFullWindow)
{
  RegisteredCloudWindow window(1000, 100);
  EXPECT_TRUE(window.push(0, cloud(4)).accepted);
  ASSERT_EQ(window.snapshot().size(), 1U);
  EXPECT_EQ(window.pointCount(), 4U);
}

TEST(RegisteredCloudWindow, SnapshotRetainsOverlappingHistoryAndExpiresOldFrames)
{
  RegisteredCloudWindow window(100, 100);
  window.push(100, cloud(4, 1.0F));
  window.push(150, cloud(5, 2.0F));
  const auto first_registration = window.snapshot();
  EXPECT_EQ(first_registration.size(), 2U);
  window.push(200, cloud(6, 3.0F));
  EXPECT_EQ(window.snapshot().size(), 3U);  // Inclusive window boundary.
  window.push(201, cloud(7, 4.0F));
  const auto next_registration = window.snapshot();
  ASSERT_EQ(next_registration.size(), 3U);
  EXPECT_EQ(next_registration.front().stamp_ns, 150);
  EXPECT_EQ(window.pointCount(), 18U);
  // Registration snapshots stay valid when the input window advances.
  EXPECT_FLOAT_EQ(first_registration.front().cloud->front().x, 1.0F);
}

TEST(RegisteredCloudWindow, PointLimitDropsOldestWholeFrames)
{
  RegisteredCloudWindow window(1000, 10);
  window.push(100, cloud(4));
  window.push(200, cloud(5));
  window.push(300, cloud(6));
  ASSERT_EQ(window.snapshot().size(), 1U);
  EXPECT_EQ(window.snapshot().front().stamp_ns, 300);
  EXPECT_EQ(window.pointCount(), 6U);
}

TEST(RegisteredCloudWindow, EmptyOversizedAndDuplicateMessagesDoNotReplaceValidHistory)
{
  RegisteredCloudWindow window(1000, 10);
  window.push(100, cloud(4));
  EXPECT_FALSE(window.push(200, cloud(0)).accepted);
  EXPECT_FALSE(window.push(200, cloud(11)).accepted);
  EXPECT_FALSE(window.push(100, cloud(4)).accepted);
  ASSERT_EQ(window.snapshot().size(), 1U);
  EXPECT_EQ(window.pointCount(), 4U);
}

TEST(RegisteredCloudWindow, TimeRewindStartsNewEpoch)
{
  RegisteredCloudWindow window(1000, 100);
  window.push(200, cloud(4));
  const auto update = window.push(10, cloud(5));
  EXPECT_TRUE(update.accepted);
  EXPECT_TRUE(update.time_reset);
  ASSERT_EQ(window.snapshot().size(), 1U);
  EXPECT_EQ(window.snapshot().front().stamp_ns, 10);
  EXPECT_EQ(window.pointCount(), 5U);
}

TEST(RegisteredCloudWindow, InputSilenceExpiresWindowWithoutNewFrame)
{
  RegisteredCloudWindow window(100, 100);
  window.push(100, cloud(4));
  window.pruneBefore(101);
  EXPECT_TRUE(window.snapshot().empty());
  EXPECT_EQ(window.pointCount(), 0U);
  EXPECT_FALSE(window.push(100, cloud(4)).accepted);
  const auto update = window.push(10, cloud(3));
  EXPECT_TRUE(update.time_reset);
  EXPECT_EQ(window.pointCount(), 3U);
}

}  // namespace
}  // namespace small_gicp_relocalization
