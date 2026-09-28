#ifndef EDGE_FLEET_CORE_P2P_PEER_NODE_HPP_
#define EDGE_FLEET_CORE_P2P_PEER_NODE_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "edge_fleet_msgs/msg/robot_intent.hpp"

namespace edge_fleet_core
{

struct PeerState
{
  edge_fleet_msgs::msg::RobotIntent intent;
  rclcpp::Time last_heartbeat;
  bool is_active{true};
};

class P2PPeerNode : public rclcpp::Node
{
public:
  explicit P2PPeerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_odom_received(const nav_msgs::msg::Odometry::SharedPtr msg);
  void on_path_received(const nav_msgs::msg::Path::SharedPtr msg);
  void on_peer_intent_received(const edge_fleet_msgs::msg::RobotIntent::SharedPtr msg);
  void on_aisle_blocked(const std_msgs::msg::Bool::SharedPtr msg);
  void on_assigned_task(const std_msgs::msg::String::SharedPtr msg);
  void broadcast_timer_callback();
  void health_check_callback();

  // Parameters
  std::string robot_id_;
  std::string broadcast_topic_;
  double broadcast_rate_hz_;
  double heartbeat_timeout_sec_;
  double battery_drain_rate_;
  float current_battery_soc_{100.0f};
  bool has_received_odom_{false};

  // State
  geometry_msgs::msg::Pose2D current_pose_;
  geometry_msgs::msg::Twist current_velocity_;
  std::vector<geometry_msgs::msg::Pose2D> predicted_path_;
  std::string current_task_id_{"TASK_IDLE"};
  uint8_t priority_level_{5};
  bool is_yielding_{false};
  bool is_blocked_{false};

  std::mutex state_mutex_;
  std::mutex peers_mutex_;
  std::unordered_map<std::string, PeerState> peers_cache_;

  // ROS 2 Interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<edge_fleet_msgs::msg::RobotIntent>::SharedPtr p2p_intent_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr aisle_blocked_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr assigned_task_sub_;

  rclcpp::Publisher<edge_fleet_msgs::msg::RobotIntent>::SharedPtr p2p_intent_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_pub_;

  rclcpp::TimerBase::SharedPtr broadcast_timer_;
  rclcpp::TimerBase::SharedPtr health_timer_;
};

}  // namespace edge_fleet_core

#endif  // EDGE_FLEET_CORE_P2P_PEER_NODE_HPP_
