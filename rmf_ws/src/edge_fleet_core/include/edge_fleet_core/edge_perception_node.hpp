#ifndef EDGE_FLEET_CORE_EDGE_PERCEPTION_NODE_HPP_
#define EDGE_FLEET_CORE_EDGE_PERCEPTION_NODE_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace edge_fleet_core
{

class EdgePerceptionNode : public rclcpp::Node
{
public:
  explicit EdgePerceptionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void on_manual_blockage_trigger(const std_msgs::msg::Bool::SharedPtr msg);
  void evaluate_perception_callback();

  std::string robot_id_;
  double forward_angle_range_deg_{35.0};
  double blockage_distance_threshold_{1.8};
  int min_cluster_points_{5};

  std::mutex mutex_;
  bool manual_blocked_override_{false};
  bool is_currently_blocked_{false};
  int continuous_blocked_frames_{0};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manual_trigger_sub_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr aisle_blocked_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  rclcpp::TimerBase::SharedPtr eval_timer_;
};

}  // namespace edge_fleet_core

#endif  // EDGE_FLEET_CORE_EDGE_PERCEPTION_NODE_HPP_
