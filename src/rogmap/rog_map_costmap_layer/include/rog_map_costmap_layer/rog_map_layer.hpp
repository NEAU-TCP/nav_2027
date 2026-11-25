#ifndef ROG_MAP_COSTMAP_LAYER__ROG_MAP_LAYER_HPP_
#define ROG_MAP_COSTMAP_LAYER__ROG_MAP_LAYER_HPP_

#include <mutex>
#include <string>

#include <nav2_costmap_2d/layer.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace rog_map_costmap_layer
{

class RogMapLayer : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y,
    double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override {return false;}

private:
  void mapCallback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;
  std::mutex mutex_;
  std::string topic_;
  int occupied_threshold_{50};
  bool enabled_{true};
};

}  // namespace rog_map_costmap_layer

#endif
