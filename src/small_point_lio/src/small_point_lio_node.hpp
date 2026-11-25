#pragma once

#include "common/common.h"
#include "small_point_lio/small_point_lio.h"

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <vector>


namespace small_point_lio {

class SmallPointLioNode final : public rclcpp::Node {
public:
  explicit SmallPointLioNode(const rclcpp::NodeOptions & options);

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void livoxCallback(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void publishOdometry(const common::Odometry & odometry);
  void publishPointCloud(const std::vector<Eigen::Vector3f> & points);

  std::unique_ptr<SmallPointLio> lio_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_full_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_body_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  std::string output_frame_;
  std::string child_frame_;
  double point_time_scale_{1.0};
  bool point_time_relative_{true};
  double cloud_batch_period_{0.1};
  common::Odometry last_odom_{};
  bool have_odom_{false};
  std::vector<Eigen::Vector3f> pending_cloud_points_;
  double pending_cloud_start_time_{0.0};
  builtin_interfaces::msg::Time pending_cloud_stamp_;
  bool has_pending_cloud_start_{false};
};

}  // namespace small_point_lio
