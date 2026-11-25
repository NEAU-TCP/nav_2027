#include "rog_map_costmap_layer/rog_map_layer.hpp"

#include <algorithm>
#include <functional>
#include <limits>

#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace rog_map_costmap_layer
{

void RogMapLayer::onInitialize()
{
  auto node = node_.lock();
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("topic", rclcpp::ParameterValue(std::string("/rog_map/layer_value")));
  declareParameter("occupied_threshold", rclcpp::ParameterValue(50));
  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".topic", topic_);
  node->get_parameter(name_ + ".occupied_threshold", occupied_threshold_);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    topic_, rclcpp::QoS(1).best_effort(),
    std::bind(&RogMapLayer::mapCallback, this, std::placeholders::_1));
  current_ = false;
}

void RogMapLayer::mapCallback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  map_ = std::move(msg);
  current_ = false;
}

void RogMapLayer::updateBounds(
  double robot_x, double robot_y, double, double * min_x, double * min_y, double * max_x,
  double * max_y)
{
  if (!enabled_) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!map_) {
    return;
  }
  // The projection is a rolling map. Its frame can differ from the costmap's
  // global frame, so update the robot-centered window and let updateCosts do
  // the exact TF conversion for each cell.
  const double range = std::max(
    map_->info.width * map_->info.resolution, map_->info.height * map_->info.resolution);
  const double x0 = robot_x - range;
  const double y0 = robot_y - range;
  const double x1 = robot_x + range;
  const double y1 = robot_y + range;
  *min_x = std::min(*min_x, x0);
  *min_y = std::min(*min_y, y0);
  *max_x = std::max(*max_x, x1);
  *max_y = std::max(*max_y, y1);
}

void RogMapLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    map = map_;
  }
  if (!map) {return;}

  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(
      layered_costmap_->getGlobalFrameID(), map->header.frame_id,
      tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    return;
  }

  if (!map->data.empty()) {
    for (unsigned int y = 0; y < map->info.height; ++y) {
      for (unsigned int x = 0; x < map->info.width; ++x) {
        const size_t index = static_cast<size_t>(y) * map->info.width + x;
        if (index >= map->data.size() || map->data[index] < occupied_threshold_) {continue;}
        geometry_msgs::msg::PointStamped source, target;
        source.header = map->header;
        source.point.x = map->info.origin.position.x + (x + 0.5) * map->info.resolution;
        source.point.y = map->info.origin.position.y + (y + 0.5) * map->info.resolution;
        tf2::doTransform(source, target, tf);
        unsigned int mx, my;
        if (master_grid.worldToMap(target.point.x, target.point.y, mx, my) &&
          static_cast<int>(mx) >= min_i && static_cast<int>(mx) < max_i &&
          static_cast<int>(my) >= min_j && static_cast<int>(my) < max_j)
        {
          master_grid.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
        }
      }
    }
  }
  current_ = true;
}

void RogMapLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  map_.reset();
  current_ = false;
}

}  // namespace rog_map_costmap_layer

PLUGINLIB_EXPORT_CLASS(rog_map_costmap_layer::RogMapLayer, nav2_costmap_2d::Layer)
