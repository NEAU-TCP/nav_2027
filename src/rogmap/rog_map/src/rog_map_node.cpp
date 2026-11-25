#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>

#include "rog_map/rog_map_core/config.hpp"
#include "rog_map_ros/rog_map_ros2.hpp"

class RogMapNode : public rclcpp::Node
{
public:
  RogMapNode()
  : Node("rog_map")
  {
    init_timer_ = create_wall_timer(std::chrono::milliseconds(1), [this]() { initialize(); });
  }

private:
  void initialize()
  {
    if (map_) {
      return;
    }
    init_timer_->cancel();
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    rog_map::Config config;
    config.loadFromRosNode(shared_from_this(), "");
    map_ = std::make_shared<rog_map::ROGMapROS>(shared_from_this(), config, tf_buffer);
    RCLCPP_INFO(get_logger(), "ROG-Map terrain and ESDF node started");
  }

  rclcpp::TimerBase::SharedPtr init_timer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<rog_map::ROGMapROS> map_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RogMapNode>());
  rclcpp::shutdown();
  return 0;
}
