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

#include "small_gicp_relocalization/small_gicp_relocalization.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "pcl/common/transforms.h"
#include "pcl_conversions/pcl_conversions.h"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"


namespace small_gicp_relocalization
{
SmallGicpRelocalizationNode::SmallGicpRelocalizationNode(const rclcpp::NodeOptions & options)
: Node("small_gicp_relocalization", options),
  result_t_(Eigen::Isometry3d::Identity()),
  previous_result_t_(Eigen::Isometry3d::Identity())
{
  this->declare_parameter("num_threads", 4);
  this->declare_parameter("num_neighbors", 20);
  this->declare_parameter("global_leaf_size", 0.25);
  this->declare_parameter("registered_leaf_size", 0.25);
  this->declare_parameter("max_dist_sq", 1.0);
  max_iterations_ = this->declare_parameter("max_iterations", 30);
  if (max_iterations_ <= 0) {
    throw std::invalid_argument("GICP max_iterations must be positive");
  }
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("base_frame", "");
  this->declare_parameter("robot_base_frame", "");
  this->declare_parameter("lidar_frame", "");
  this->declare_parameter("prior_pcd_file", "");
  this->declare_parameter("init_pose", std::vector<double>{0., 0., 0., 0., 0., 0.});
  this->declare_parameter("input_cloud_topic", "registered_scan");
  this->declare_parameter("cloud_window_duration", 1.0);
  this->declare_parameter("max_window_points", 600000);
  quality_limits_.min_overlap = this->declare_parameter("min_overlap", 0.4);
  quality_limits_.max_rmse = this->declare_parameter("max_rmse", 0.2);
  quality_limits_.max_translation_delta = this->declare_parameter("max_translation_delta", 1.0);
  quality_limits_.max_rotation_delta = this->declare_parameter("max_rotation_delta", 0.35);
  if (!std::isfinite(quality_limits_.min_overlap) || quality_limits_.min_overlap <= 0.0 ||
    quality_limits_.min_overlap > 1.0 || !std::isfinite(quality_limits_.max_rmse) ||
    quality_limits_.max_rmse <= 0.0 || !std::isfinite(quality_limits_.max_translation_delta) ||
    quality_limits_.max_translation_delta <= 0.0 ||
    !std::isfinite(quality_limits_.max_rotation_delta) || quality_limits_.max_rotation_delta <= 0.0)
  {
    throw std::invalid_argument("Invalid GICP overlap, RMSE or correction jump limits");
  }

  this->get_parameter("num_threads", num_threads_);
  this->get_parameter("num_neighbors", num_neighbors_);
  this->get_parameter("global_leaf_size", global_leaf_size_);
  this->get_parameter("registered_leaf_size", registered_leaf_size_);
  this->get_parameter("max_dist_sq", max_dist_sq_);
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("prior_pcd_file", prior_pcd_file_);
  this->get_parameter("init_pose", init_pose_);
  this->get_parameter("input_cloud_topic", input_cloud_topic_);
  this->get_parameter("cloud_window_duration", cloud_window_duration_);
  this->get_parameter("max_window_points", max_window_points_);
  if (!std::isfinite(cloud_window_duration_) || cloud_window_duration_ <= 0.0 ||
    cloud_window_duration_ > 60.0 || max_window_points_ <= 0)
  {
    throw std::invalid_argument(
            "cloud_window_duration must be in (0, 60] seconds and max_window_points must be positive");
  }

  // [x, y, z, roll, pitch, yaw] - init_pose parameters
  if (!init_pose_.empty() && init_pose_.size() >= 6) {
    result_t_.translation() << init_pose_[0], init_pose_[1], init_pose_[2];
    result_t_.linear() =
      Eigen::AngleAxisd(init_pose_[5], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(init_pose_[4], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(init_pose_[3], Eigen::Vector3d::UnitX()).toRotationMatrix();
  }
  previous_result_t_ = result_t_;
  initial_result_t_ = result_t_;

  cloud_window_ = std::make_unique<RegisteredCloudWindow>(
    static_cast<int64_t>(cloud_window_duration_ * 1e9),
    static_cast<size_t>(max_window_points_));
  global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  register_ = std::make_shared<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  loadGlobalMap(prior_pcd_file_);

  // Build the registration target asynchronously so loading the component does
  // not block the container while waiting for static TF or downsampling the map.
  map_init_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&SmallGicpRelocalizationNode::initializeGlobalMap, this));

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_, rclcpp::SensorDataQoS(),
    std::bind(&SmallGicpRelocalizationNode::registeredPcdCallback, this, std::placeholders::_1));

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10,
    std::bind(&SmallGicpRelocalizationNode::initialPoseCallback, this, std::placeholders::_1));

  register_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),  // 2 Hz
    std::bind(&SmallGicpRelocalizationNode::performRegistration, this));

  transform_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),  // 20 Hz
    std::bind(&SmallGicpRelocalizationNode::publishTransform, this));

  RCLCPP_INFO(
    this->get_logger(),
    "GICP sliding window: topic=%s frame=%s duration=%.2fs max_points=%d registration_period=0.50s",
    input_cloud_topic_.c_str(), odom_frame_.c_str(), cloud_window_duration_, max_window_points_);
}

void SmallGicpRelocalizationNode::loadGlobalMap(const std::string & file_name)
{
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s", file_name.c_str());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Loaded global map %s with %zu points", file_name.c_str(), global_map_->points.size());
}

void SmallGicpRelocalizationNode::initializeGlobalMap()
{
  if (global_map_->empty() || target_tree_) {
    if (global_map_->empty() && map_init_timer_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Global map is empty; GICP target initialization is disabled");
      map_init_timer_->cancel();
    }
    return;
  }

  try {
    // The prior PCD is saved in the LIO lidar_odom frame. registered_scan is
    // already in odom, so convert the prior map using the startup lidar extrinsic.
    const auto tf_stamped = tf_buffer_->lookupTransform(
      base_frame_, lidar_frame_, tf2::TimePointZero, tf2::durationFromSec(0.1));
    const Eigen::Affine3d base_to_lidar = tf2::transformToEigen(tf_stamped.transform);
    pcl::transformPointCloud(*global_map_, *global_map_, base_to_lidar);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Waiting for TF %s <- %s to convert the prior map into %s: %s",
      base_frame_.c_str(), lidar_frame_.c_str(), odom_frame_.c_str(), ex.what());
    return;
  }

  target_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *global_map_, global_leaf_size_);
  small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);
  target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    target_, small_gicp::KdTreeBuilderOMP(num_threads_));

  RCLCPP_INFO(
    this->get_logger(), "GICP target initialized in %s with %zu downsampled points",
    map_frame_.c_str(), target_->points.size());
  map_init_timer_->cancel();
}

void SmallGicpRelocalizationNode::registeredPcdCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (msg->header.frame_id != odom_frame_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Rejecting GICP input frame '%s': sliding window requires registered clouds in '%s'",
      msg->header.frame_id.c_str(), odom_frame_.c_str());
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *scan);
  // Reject invalid coordinates before voxel hashing and covariance estimation.
  scan->erase(std::remove_if(scan->begin(), scan->end(), [](const pcl::PointXYZ & point) {
    return !std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z);
  }), scan->end());
  if (scan->size() > static_cast<size_t>(max_window_points_)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "GICP input batch has %zu points, exceeding max_window_points=%d; skipping batch",
      scan->size(), max_window_points_);
    return;
  }
  {
    std::lock_guard<std::mutex> lock(input_cloud_mutex_);
    const auto update = cloud_window_->push(rclcpp::Time(msg->header.stamp).nanoseconds(), scan);
    if (!update.accepted) {
      return;
    }
    current_scan_frame_id_ = msg->header.frame_id;
    window_updated_ = true;
    if (update.time_reset) {
      // A simulation restart starts a new odom epoch. Never mix the two maps.
      has_valid_transform_ = false;
      result_t_ = previous_result_t_ = initial_result_t_;
      RCLCPP_WARN(this->get_logger(), "GICP cloud time moved backwards; reset sliding window and correction");
    }
  }
}

void SmallGicpRelocalizationNode::performRegistration()
{
  if (!target_ || !target_tree_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "GICP target is not initialized yet; waiting for the prior map target");
    return;
  }
  std::vector<RegisteredCloudWindow::Frame> frames;
  size_t point_count = 0;
  {
    std::lock_guard<std::mutex> lock(input_cloud_mutex_);
    cloud_window_->pruneBefore(
      this->now().nanoseconds() - static_cast<int64_t>(cloud_window_duration_ * 1e9));
    if (!window_updated_) {
      return;
    }
    window_updated_ = false;
    frames = cloud_window_->snapshot();
    point_count = cloud_window_->pointCount();
  }
  if (frames.empty()) {
    return;
  }
  // Only copy at registration frequency, outside the input-window lock.
  pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
  scan->reserve(point_count);
  for (const auto & frame : frames) {
    *scan += *frame.cloud;
  }

  source_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *scan, registered_leaf_size_);

  if (source_->size() < static_cast<size_t>(std::max(20, num_neighbors_))) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "GICP window has too few downsampled points (%zu); waiting for more input", source_->size());
    return;
  }
  const double span = static_cast<double>(frames.back().stamp_ns - frames.front().stamp_ns) * 1e-9;
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 5000,
    "GICP window: frames=%zu span=%.3fs input=%zu downsampled=%zu",
    frames.size(), span, scan->size(), source_->size());

  small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);

  source_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    source_, small_gicp::KdTreeBuilderOMP(num_threads_));

  if (!source_ || !source_tree_) {
    return;
  }

  register_->reduction.num_threads = num_threads_;
  register_->rejector.max_dist_sq = max_dist_sq_;
  register_->optimizer.max_iterations = max_iterations_;

  auto result = register_->align(*target_, *source_, *target_tree_, previous_result_t_);

  std::vector<double> squared_distances(source_->size(), std::numeric_limits<double>::infinity());
  if (result.T_target_source.matrix().allFinite()) {
    for (size_t i = 0; i < source_->size(); ++i) {
      const auto & point = source_->points[i];
      const Eigen::Vector4d transformed = result.T_target_source *
        Eigen::Vector4d(point.x, point.y, point.z, 1.0);
      size_t neighbor;
      double distance;
      if (target_tree_->nearest_neighbor_search(transformed, &neighbor, &distance)) {
        squared_distances[i] = distance;
      }
    }
  }
  const auto quality = assessRegistration(
    result.converged, result.T_target_source, previous_result_t_, squared_distances, quality_limits_);
  if (!quality.rejection) {
    result_t_ = previous_result_t_ = result.T_target_source;
    has_valid_transform_ = true;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "GICP accepted: overlap=%.3f rmse=%.3fm correction_delta=%.3fm/%.3frad map_to_odom=(%.3f,%.3f,%.3f)",
      quality.overlap, quality.rmse, quality.translation_delta, quality.rotation_delta,
      result_t_.translation().x(), result_t_.translation().y(), result_t_.translation().z());
  } else {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "GICP rejected (%s): overlap=%.3f rmse=%.3fm correction_delta=%.3fm/%.3frad converged=%d iterations=%zu inliers=%zu; keeping last accepted TF",
      quality.rejection, quality.overlap, quality.rmse, quality.translation_delta,
      quality.rotation_delta, result.converged, result.iterations, result.num_inliers);
  }
}

void SmallGicpRelocalizationNode::publishTransform()
{
  if (!has_valid_transform_) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = this->now();
  transform_stamped.header.frame_id = map_frame_;
  transform_stamped.child_frame_id = odom_frame_;

  const Eigen::Vector3d translation = result_t_.translation();
  const Eigen::Quaterniond rotation(result_t_.rotation());

  transform_stamped.transform.translation.x = translation.x();
  transform_stamped.transform.translation.y = translation.y();
  transform_stamped.transform.translation.z = translation.z();
  transform_stamped.transform.rotation.x = rotation.x();
  transform_stamped.transform.rotation.y = rotation.y();
  transform_stamped.transform.rotation.z = rotation.z();
  transform_stamped.transform.rotation.w = rotation.w();

  tf_broadcaster_->sendTransform(transform_stamped);
}

void SmallGicpRelocalizationNode::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  const auto & orientation = msg->pose.pose.orientation;
  Eigen::Quaterniond rotation(orientation.w, orientation.x, orientation.y, orientation.z);
  const auto & position = msg->pose.pose.position;
  if (msg->header.frame_id != map_frame_ || !rotation.coeffs().allFinite() ||
    rotation.norm() < 1e-6 || !std::isfinite(position.x) || !std::isfinite(position.y) ||
    !std::isfinite(position.z))
  {
    RCLCPP_WARN(get_logger(), "Rejecting invalid initial pose: expected finite pose in %s", map_frame_.c_str());
    return;
  }
  RCLCPP_INFO(
    this->get_logger(), "Received initial pose: [x: %f, y: %f, z: %f]", msg->pose.pose.position.x,
    msg->pose.pose.position.y, msg->pose.pose.position.z);

  Eigen::Isometry3d map_to_robot_base = Eigen::Isometry3d::Identity();
  map_to_robot_base.translation() << msg->pose.pose.position.x, msg->pose.pose.position.y,
    msg->pose.pose.position.z;
  map_to_robot_base.linear() = rotation.normalized().toRotationMatrix();

  try {
    auto transform =
      tf_buffer_->lookupTransform(robot_base_frame_, current_scan_frame_id_, tf2::TimePointZero);
    Eigen::Isometry3d robot_base_to_odom = tf2::transformToEigen(transform.transform);
    Eigen::Isometry3d map_to_odom = map_to_robot_base * robot_base_to_odom;

    previous_result_t_ = result_t_ = map_to_odom;
    has_valid_transform_ = true;
    // Retry the retained window against the explicitly supplied initial guess.
    window_updated_ = true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "Could not transform initial pose from %s to %s: %s",
      robot_base_frame_.c_str(), current_scan_frame_id_.c_str(), ex.what());
  }
}

}  // namespace small_gicp_relocalization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(small_gicp_relocalization::SmallGicpRelocalizationNode)
