#include "small_point_lio_node.hpp"

#include <cstring>
#include <limits>
#include <stdexcept>

namespace small_point_lio {
namespace {

builtin_interfaces::msg::Time timeMessage(double seconds)
{
  builtin_interfaces::msg::Time msg;
  const auto sec = static_cast<int64_t>(std::floor(seconds));
  msg.sec = static_cast<int32_t>(sec);
  msg.nanosec = static_cast<uint32_t>((seconds - static_cast<double>(sec)) * 1e9);
  return msg;
}

const sensor_msgs::msg::PointField * findField(
  const sensor_msgs::msg::PointCloud2 & msg, const std::string & name)
{
  for (const auto & field : msg.fields) {
    if (field.name == name) {
      return &field;
    }
  }
  return nullptr;
}

double readField(const uint8_t * data, const sensor_msgs::msg::PointField & field)
{
  switch (field.datatype) {
    case sensor_msgs::msg::PointField::FLOAT32: {
      float value;
      std::memcpy(&value, data + field.offset, sizeof(value));
      return value;
    }
    case sensor_msgs::msg::PointField::FLOAT64: {
      double value;
      std::memcpy(&value, data + field.offset, sizeof(value));
      return value;
    }
    case sensor_msgs::msg::PointField::UINT8: return *(data + field.offset);
    case sensor_msgs::msg::PointField::UINT16: {
      uint16_t value;
      std::memcpy(&value, data + field.offset, sizeof(value));
      return value;
    }
    case sensor_msgs::msg::PointField::UINT32: {
      uint32_t value;
      std::memcpy(&value, data + field.offset, sizeof(value));
      return value;
    }
    case sensor_msgs::msg::PointField::INT8: return *reinterpret_cast<const int8_t *>(data + field.offset);
    case sensor_msgs::msg::PointField::INT16: {
      int16_t value;
      std::memcpy(&value, data + field.offset, sizeof(value));
      return value;
    }
    case sensor_msgs::msg::PointField::INT32: {
      int32_t value;
      std::memcpy(&value, data + field.offset, sizeof(value));
      return value;
    }
    default: return std::numeric_limits<double>::quiet_NaN();
  }
}

sensor_msgs::msg::PointCloud2 makeCloud(
  const std::vector<Eigen::Vector3f> & points, const builtin_interfaces::msg::Time & stamp,
  const std::string & frame)
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame;
  msg.height = 1;
  msg.width = static_cast<uint32_t>(points.size());
  msg.is_bigendian = false;
  msg.is_dense = false;
  msg.point_step = 16;
  msg.row_step = msg.point_step * msg.width;
  msg.fields.resize(4);
  const char * names[] = {"x", "y", "z", "intensity"};
  for (size_t i = 0; i < 4; ++i) {
    msg.fields[i].name = names[i];
    msg.fields[i].offset = static_cast<uint32_t>(i * sizeof(float));
    msg.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[i].count = 1;
  }
  msg.data.resize(msg.row_step);
  for (size_t i = 0; i < points.size(); ++i) {
    float values[4] = {points[i].x(), points[i].y(), points[i].z(), 0.0F};
    std::memcpy(msg.data.data() + i * msg.point_step, values, sizeof(values));
  }
  return msg;
}

}  // namespace

SmallPointLioNode::SmallPointLioNode(const rclcpp::NodeOptions & options)
: Node("small_point_lio", options)
{
  const auto lidar_topic = declare_parameter<std::string>("lidar_topic", "velodyne_points");
  const auto imu_topic = declare_parameter<std::string>("imu_topic", "livox/imu");
  const auto lidar_type = declare_parameter<std::string>("lidar_type", "pointcloud2");
  output_frame_ = declare_parameter<std::string>("output_frame", "camera_init");
  child_frame_ = declare_parameter<std::string>("child_frame", "front_mid360");
  const auto full_topic = declare_parameter<std::string>("cloud_registered_full_topic", "cloud_registered_full");
  const auto cloud_topic = declare_parameter<std::string>("cloud_registered_topic", "cloud_registered");
  const auto body_topic = declare_parameter<std::string>("cloud_registered_body_topic", "cloud_registered_body");
  const auto odometry_topic = declare_parameter<std::string>("odometry_topic", "aft_mapped_to_init");
  point_time_scale_ = declare_parameter<double>("point_time_scale", 1.0);
  point_time_relative_ = declare_parameter<bool>("point_time_relative", true);
  cloud_batch_period_ = declare_parameter<double>("cloud_batch_period", 0.1);
  if (cloud_batch_period_ <= 0.0 || cloud_batch_period_ > 1.0) {
    throw std::invalid_argument("cloud_batch_period must be in (0, 1] seconds");
  }
  // Small Point-LIO owns the estimator only. TF remains owned by the existing
  // robot_state_publisher/loam_interface chain to keep the project's TF tree stable.
  cloud_full_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(full_topic, rclcpp::SensorDataQoS().keep_last(1));
  cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic, rclcpp::SensorDataQoS().keep_last(1));
  cloud_body_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(body_topic, rclcpp::SensorDataQoS().keep_last(1));
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odometry_topic, 20);

  lio_ = std::make_unique<SmallPointLio>(*this);
  lio_->set_odometry_callback([this](const common::Odometry & odometry) { publishOdometry(odometry); });
  lio_->set_pointcloud_callback([this](const std::vector<Eigen::Vector3f> & points) { publishPointCloud(points); });

  if (lidar_type == "livox_custom_msg") {
    livox_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      lidar_topic, rclcpp::SensorDataQoS(),
      std::bind(&SmallPointLioNode::livoxCallback, this, std::placeholders::_1));
  } else {
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic, rclcpp::SensorDataQoS(),
      std::bind(&SmallPointLioNode::pointCloudCallback, this, std::placeholders::_1));
  }
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, rclcpp::SensorDataQoS(),
    std::bind(&SmallPointLioNode::imuCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Small Point-LIO input: cloud=%s imu=%s output_frame=%s", lidar_topic.c_str(),
    imu_topic.c_str(), output_frame_.c_str());
  RCLCPP_INFO(get_logger(), "Registered cloud short-batch period: %.3fs", cloud_batch_period_);
}

void SmallPointLioNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  const auto * x_field = findField(*msg, "x");
  const auto * y_field = findField(*msg, "y");
  const auto * z_field = findField(*msg, "z");
  const auto * time_field = findField(*msg, "time");
  const auto * timestamp_field = time_field ? time_field : findField(*msg, "timestamp");
  if (!x_field || !y_field || !z_field) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "PointCloud2 has no x/y/z fields");
    return;
  }
  std::vector<common::Point> points;
  points.reserve(static_cast<size_t>(msg->width) * msg->height);
  const double stamp = rclcpp::Time(msg->header.stamp).seconds();
  const size_t count = static_cast<size_t>(msg->width) * msg->height;
  for (size_t i = 0; i < count; ++i) {
    const auto * point_data = msg->data.data() + i * msg->point_step;
    const double x = readField(point_data, *x_field);
    const double y = readField(point_data, *y_field);
    const double z = readField(point_data, *z_field);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    double point_time = 0.0;
    if (timestamp_field) {
      point_time = readField(point_data, *timestamp_field) * point_time_scale_;
    }
    if (point_time_relative_) {
      point_time += stamp;
    }
    points.push_back({point_time, Eigen::Vector3f(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z))});
  }
  if (!points.empty()) {
    lio_->on_point_cloud_callback(points);
    lio_->handle_once();
  }
}

void SmallPointLioNode::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  common::ImuMsg imu;
  imu.timestamp = rclcpp::Time(msg->header.stamp).seconds();
  imu.angular_velocity = Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  imu.linear_acceleration = Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  lio_->on_imu_callback(imu);
  lio_->handle_once();
}

void SmallPointLioNode::livoxCallback(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg)
{
  const double timebase = static_cast<double>(msg->timebase) * 1e-9;
  std::vector<common::Point> points;
  points.reserve(msg->point_num);
  for (const auto & raw : msg->points) {
    points.push_back({timebase + static_cast<double>(raw.offset_time) * 1e-9,
      Eigen::Vector3f(raw.x, raw.y, raw.z)});
  }
  if (!points.empty()) {
    lio_->on_point_cloud_callback(points);
    lio_->handle_once();
  }
}

void SmallPointLioNode::publishOdometry(const common::Odometry & odometry)
{
  last_odom_ = odometry;
  have_odom_ = true;
  nav_msgs::msg::Odometry msg;
  msg.header.stamp = timeMessage(odometry.timestamp);
  msg.header.frame_id = output_frame_;
  msg.child_frame_id = child_frame_;
  msg.pose.pose.position.x = odometry.position.x();
  msg.pose.pose.position.y = odometry.position.y();
  msg.pose.pose.position.z = odometry.position.z();
  msg.pose.pose.orientation.x = odometry.orientation.x();
  msg.pose.pose.orientation.y = odometry.orientation.y();
  msg.pose.pose.orientation.z = odometry.orientation.z();
  msg.pose.pose.orientation.w = odometry.orientation.w();
  msg.twist.twist.linear.x = odometry.velocity.x();
  msg.twist.twist.linear.y = odometry.velocity.y();
  msg.twist.twist.linear.z = odometry.velocity.z();
  msg.twist.twist.angular.x = odometry.angular_velocity.x();
  msg.twist.twist.angular.y = odometry.angular_velocity.y();
  msg.twist.twist.angular.z = odometry.angular_velocity.z();
  odom_pub_->publish(msg);
}

void SmallPointLioNode::publishPointCloud(const std::vector<Eigen::Vector3f> & points)
{
  if (!have_odom_ || points.empty()) {
    return;
  }

  const auto stamp = timeMessage(last_odom_.timestamp);
  const double stamp_seconds = last_odom_.timestamp;
  if (has_pending_cloud_start_ &&
    stamp_seconds < rclcpp::Time(pending_cloud_stamp_).seconds())
  {
    // Never carry pending output across a simulation time reset.
    pending_cloud_points_.clear();
    has_pending_cloud_start_ = false;
  }

  if (!has_pending_cloud_start_) {
    pending_cloud_start_time_ = stamp_seconds;
    has_pending_cloud_start_ = true;
  }
  pending_cloud_points_.insert(pending_cloud_points_.end(), points.begin(), points.end());
  pending_cloud_stamp_ = stamp;
  if (stamp_seconds - pending_cloud_start_time_ >= cloud_batch_period_) {
    // Points already have per-point motion compensation in the fixed LIO frame.
    // Only output is batched; estimator and odometry updates remain per point.
    auto msg = makeCloud(pending_cloud_points_, pending_cloud_stamp_, output_frame_);
    cloud_full_pub_->publish(msg);
    cloud_pub_->publish(msg);
    cloud_body_pub_->publish(msg);
    pending_cloud_points_.clear();
    pending_cloud_start_time_ = stamp_seconds;
  }
}

}  // namespace small_point_lio

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(small_point_lio::SmallPointLioNode)

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<small_point_lio::SmallPointLioNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
