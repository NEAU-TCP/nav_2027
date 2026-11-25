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

#include "sensor_scan_generation/sensor_scan_generation.hpp"

#include "pcl_ros/transforms.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace sensor_scan_generation
{

SensorScanGenerationNode::SensorScanGenerationNode(const rclcpp::NodeOptions & options)
: Node("sensor_scan_generation", options)
{
  this->declare_parameter<std::string>("lidar_frame", "");
  this->declare_parameter<std::string>("base_frame", "");
  this->declare_parameter<std::string>("robot_base_frame", "");

  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  pub_laser_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sensor_scan", 2);
  pub_chassis_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 2);

  // LIO odometry is per point, whereas clouds are batched. TF must not wait for
  // an ApproximateTime pair whose odometry has already fallen out of the queue.
  odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "lidar_odometry", rclcpp::SensorDataQoS().keep_last(20),
    std::bind(&SensorScanGenerationNode::odometryCallback, this, std::placeholders::_1));
  laser_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "registered_scan", rclcpp::SensorDataQoS().keep_last(2),
    std::bind(&SensorScanGenerationNode::pointCloudCallback, this, std::placeholders::_1));
}

void SensorScanGenerationNode::odometryCallback(
  const nav_msgs::msg::Odometry::ConstSharedPtr odometry_msg)
{
  const int64_t stamp_ns = rclcpp::Time(odometry_msg->header.stamp).nanoseconds();
  if (have_tf_stamp_ && stamp_ns < last_tf_stamp_ns_) {
    have_tf_stamp_ = false;
    have_previous_pose_ = false;
    pending_cloud_.reset();
  }
  // Preserve the estimator's full rate upstream; publish chassis TF at <=100 Hz.
  if (have_tf_stamp_ && stamp_ns - last_tf_stamp_ns_ < 10000000) {
    return;
  }
  tf2::Transform tf_odom_to_lidar;
  tf2::fromMsg(odometry_msg->pose.pose, tf_odom_to_lidar);
  try {
    // This installation extrinsic is fixed and does not depend on cloud timing.
    const auto extrinsic = tf_buffer_->lookupTransform(
      lidar_frame_, base_frame_, tf2::TimePointZero);
    tf2::Transform tf_lidar_to_chassis;
    tf2::fromMsg(extrinsic.transform, tf_lidar_to_chassis);
    publishTransform(
      tf_odom_to_lidar * tf_lidar_to_chassis,
      odometry_msg->header.frame_id, base_frame_, odometry_msg->header.stamp);
    have_tf_stamp_ = true;
    last_tf_stamp_ns_ = stamp_ns;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Waiting for chassis extrinsic: %s", ex.what());
    return;
  }
  try {
    const auto extrinsic = tf_buffer_->lookupTransform(
      lidar_frame_, robot_base_frame_, odometry_msg->header.stamp);
    tf2::Transform tf_lidar_to_robot_base;
    tf2::fromMsg(extrinsic.transform, tf_lidar_to_robot_base);
    publishOdometry(
      tf_odom_to_lidar * tf_lidar_to_robot_base,
      odometry_msg->header.frame_id, robot_base_frame_, odometry_msg->header.stamp);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Waiting for robot-base extrinsic: %s", ex.what());
  }
  publishPendingCloud();
}

void SensorScanGenerationNode::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
  pending_cloud_ = cloud;
  publishPendingCloud();
}

void SensorScanGenerationNode::publishPendingCloud()
{
  if (!pending_cloud_) {
    return;
  }
  try {
    // Interpolate the TF history at the cloud stamp, never use a mismatched
    // latest odometry or an identity fallback. Retry after the next odom update.
    const auto transform_stamped = tf_buffer_->lookupTransform(
      lidar_frame_, pending_cloud_->header.frame_id, pending_cloud_->header.stamp);
    tf2::Transform transform;
    tf2::fromMsg(transform_stamped.transform, transform);
    sensor_msgs::msg::PointCloud2 out;
    pcl_ros::transformPointCloud(lidar_frame_, transform, *pending_cloud_, out);
    pub_laser_cloud_->publish(out);
    pending_cloud_.reset();
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Waiting for cloud-time TF: %s", ex.what());
  }
}

void SensorScanGenerationNode::publishTransform(
  const tf2::Transform & transform, const std::string & parent_frame,
  const std::string & child_frame, const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg.header.stamp = stamp;
  transform_msg.header.frame_id = parent_frame;
  transform_msg.child_frame_id = child_frame;
  transform_msg.transform = tf2::toMsg(transform);
  // Make our own history immediately available to the cloud callback, without
  // waiting for the broadcaster/listener round trip through DDS.
  tf_buffer_->setTransform(transform_msg, "sensor_scan_generation", false);
  br_->sendTransform(transform_msg);
}

void SensorScanGenerationNode::publishOdometry(
  const tf2::Transform & transform, std::string parent_frame, const std::string & child_frame,
  const rclcpp::Time & stamp)
{
  nav_msgs::msg::Odometry out;
  out.header.stamp = stamp;
  out.header.frame_id = parent_frame;
  out.child_frame_id = child_frame;

  const auto & origin = transform.getOrigin();
  out.pose.pose.position.x = origin.x();
  out.pose.pose.position.y = origin.y();
  out.pose.pose.position.z = origin.z();
  out.pose.pose.orientation = tf2::toMsg(transform.getRotation());

  const int64_t stamp_ns = stamp.nanoseconds();
  const double dt = static_cast<double>(stamp_ns - previous_pose_stamp_ns_) * 1e-9;
  if (have_previous_pose_ && dt > 0.0 && dt < 1.0) {
    const auto linear_velocity = transform.getBasis().transpose() *
      (transform.getOrigin() - previous_pose_.getOrigin()) / dt;

    tf2::Quaternion q_diff =
      previous_pose_.getRotation().inverse() * transform.getRotation();
    q_diff.normalize();
    if (q_diff.w() < 0.0) {
      q_diff = tf2::Quaternion(-q_diff.x(), -q_diff.y(), -q_diff.z(), -q_diff.w());
    }
    const auto angular_velocity = q_diff.getAxis() * q_diff.getAngle() / dt;

    out.twist.twist.linear.x = linear_velocity.x();
    out.twist.twist.linear.y = linear_velocity.y();
    out.twist.twist.linear.z = linear_velocity.z();
    out.twist.twist.angular.x = angular_velocity.x();
    out.twist.twist.angular.y = angular_velocity.y();
    out.twist.twist.angular.z = angular_velocity.z();
  }

  previous_pose_ = transform;
  previous_pose_stamp_ns_ = stamp_ns;
  have_previous_pose_ = true;

  pub_chassis_odometry_->publish(out);
}

}  // namespace sensor_scan_generation

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(sensor_scan_generation::SensorScanGenerationNode)
