#ifndef EDGE_FLEET_CORE_TASK_ALLOCATOR_NODE_HPP_
#define EDGE_FLEET_CORE_TASK_ALLOCATOR_NODE_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "edge_fleet_msgs/msg/task_auction.hpp"

namespace edge_fleet_core
{

struct BidInfo
{
  std::string bidder_id;
  float cost;
  float battery_soc;
};

class TaskAllocatorNode : public rclcpp::Node
{
public:
  explicit TaskAllocatorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Trigger an auction when an aisle is blocked
  void trigger_task_auction(
    const std::string & task_id,
    const geometry_msgs::msg::Point & pickup,
    const geometry_msgs::msg::Point & dropoff,
    const std::string & reason);

private:
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void on_auction_msg(const edge_fleet_msgs::msg::TaskAuction::SharedPtr msg);
  void on_aisle_blocked_signal(const std_msgs::msg::Bool::SharedPtr msg);
  void auction_timer_callback();

  std::string robot_id_;
  float current_battery_soc_{100.0f};
  geometry_msgs::msg::Point current_pos_;

  std::mutex mutex_;
  bool is_auction_active_{false};
  std::string active_auction_id_;
  std::string active_task_id_;
  geometry_msgs::msg::Point active_pickup_;
  geometry_msgs::msg::Point active_dropoff_;
  std::vector<BidInfo> collected_bids_;
  rclcpp::Time auction_start_time_;

  // ROS 2 Interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<edge_fleet_msgs::msg::TaskAuction>::SharedPtr auction_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr aisle_blocked_sub_;

  rclcpp::Publisher<edge_fleet_msgs::msg::TaskAuction>::SharedPtr auction_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr assigned_task_pub_;

  rclcpp::TimerBase::SharedPtr auction_eval_timer_;
};

}  // namespace edge_fleet_core

#endif  // EDGE_FLEET_CORE_TASK_ALLOCATOR_NODE_HPP_
