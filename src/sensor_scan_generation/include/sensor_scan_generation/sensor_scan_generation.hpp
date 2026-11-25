// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SENSOR_SCAN_GENERATION__SENSOR_SCAN_GENERATION_HPP_
#define SENSOR_SCAN_GENERATION__SENSOR_SCAN_GENERATION_HPP_

#include <memory>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace sensor_scan_generation
{

class SensorScanGenerationNode : public rclcpp::Node
{
public:
  explicit SensorScanGenerationNode(const rclcpp::NodeOptions & options);

private:
  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odometry);
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);
  void publishPendingCloud();

  void publishTransform(
    const tf2::Transform & transform, const std::string & parent_frame,
    const std::string & child_frame, const rclcpp::Time & stamp);

  void publishOdometry(
    const tf2::Transform & transform, std::string parent_frame, const std::string & child_frame,
    const rclcpp::Time & stamp);

  std::string lidar_frame_;
  std::string base_frame_;
  std::string robot_base_frame_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> br_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_laser_cloud_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_chassis_odometry_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr laser_cloud_sub_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pending_cloud_;
  bool have_tf_stamp_{false};
  int64_t last_tf_stamp_ns_{0};
  bool have_previous_pose_{false};
  int64_t previous_pose_stamp_ns_{0};
  tf2::Transform previous_pose_{tf2::Transform::getIdentity()};
};

}  // namespace sensor_scan_generation

#endif  // SENSOR_SCAN_GENERATION__SENSOR_SCAN_GENERATION_HPP_
