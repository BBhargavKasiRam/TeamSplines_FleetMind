#ifndef EDGE_FLEET_CORE_FLEET_BENCHMARK_NODE_HPP_
#define EDGE_FLEET_CORE_FLEET_BENCHMARK_NODE_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "edge_fleet_msgs/msg/robot_intent.hpp"
#include "edge_fleet_msgs/msg/conflict_event.hpp"
#include "edge_fleet_msgs/msg/fleet_benchmark.hpp"

namespace edge_fleet_core
{

class FleetBenchmarkNode : public rclcpp::Node
{
public:
  explicit FleetBenchmarkNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_intent(const edge_fleet_msgs::msg::RobotIntent::SharedPtr msg);
  void on_conflict_event(const edge_fleet_msgs::msg::ConflictEvent::SharedPtr msg);
  void benchmark_timer_callback();

  std::string active_mode_{"EDGE_AI"};
  double collision_dist_threshold_{0.40};  // If distance < 40cm, collision flagged

  std::mutex mutex_;
  rclcpp::Time start_time_;
  std::unordered_map<std::string, edge_fleet_msgs::msg::RobotIntent> fleet_cache_;

  uint32_t total_conflicts_count_{0};
  uint32_t dynamic_resolutions_count_{0};
  uint32_t deadlocks_prevented_count_{0};
  uint32_t collision_count_{0};
  float total_waiting_delay_sec_{0.0f};

  rclcpp::Subscription<edge_fleet_msgs::msg::RobotIntent>::SharedPtr intent_sub_;
  rclcpp::Subscription<edge_fleet_msgs::msg::ConflictEvent>::SharedPtr conflict_sub_;

  rclcpp::Publisher<edge_fleet_msgs::msg::FleetBenchmark>::SharedPtr benchmark_pub_;
  rclcpp::TimerBase::SharedPtr benchmark_timer_;
};

}  // namespace edge_fleet_core

#endif  // EDGE_FLEET_CORE_FLEET_BENCHMARK_NODE_HPP_
