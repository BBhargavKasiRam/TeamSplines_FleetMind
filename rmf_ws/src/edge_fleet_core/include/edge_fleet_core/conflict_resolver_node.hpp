#ifndef EDGE_FLEET_CORE_CONFLICT_RESOLVER_NODE_HPP_
#define EDGE_FLEET_CORE_CONFLICT_RESOLVER_NODE_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "edge_fleet_msgs/msg/robot_intent.hpp"
#include "edge_fleet_msgs/msg/conflict_event.hpp"

namespace edge_fleet_core
{

struct Vector2D
{
  double x{0.0};
  double y{0.0};

  Vector2D() = default;
  Vector2D(double _x, double _y) : x(_x), y(_y) {}

  double norm() const { return std::hypot(x, y); }
  double dot(const Vector2D & o) const { return x * o.x + y * o.y; }
  Vector2D normalized() const
  {
    double n = norm();
    return (n > 1e-6) ? Vector2D(x / n, y / n) : Vector2D(0.0, 0.0);
  }
  Vector2D operator+(const Vector2D & o) const { return Vector2D(x + o.x, y + o.y); }
  Vector2D operator-(const Vector2D & o) const { return Vector2D(x - o.x, y - o.y); }
  Vector2D operator*(double s) const { return Vector2D(x * s, y * s); }
};

class ConflictResolverNode : public rclcpp::Node
{
public:
  explicit ConflictResolverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void on_cmd_vel_raw(const geometry_msgs::msg::Twist::SharedPtr msg);
  void on_peer_intent(const edge_fleet_msgs::msg::RobotIntent::SharedPtr msg);
  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void control_loop_callback();

  // Core algorithms
  geometry_msgs::msg::Twist resolve_edge_ai_orca(const geometry_msgs::msg::Twist & desired_twist);

  // Parameters
  std::string robot_id_;
  std::string mode_{"EDGE_AI"};
  double robot_radius_{0.35};
  double safety_margin_{0.25};
  double time_horizon_{4.0};
  double max_speed_{0.8};
  double emergency_brake_dist_{0.45};
  int priority_level_{5};

  // State
  std::mutex mutex_;
  Vector2D current_pos_{0.0, 0.0};
  double current_yaw_{0.0};
  Vector2D current_vel_{0.0, 0.0};
  geometry_msgs::msg::Twist desired_twist_;
  bool has_received_desired_twist_{false};
  rclcpp::Time last_desired_twist_time_;

  double min_lidar_dist_{100.0};
  double total_waiting_time_{0.0};
  rclcpp::Time last_loop_time_;

  std::unordered_map<std::string, edge_fleet_msgs::msg::RobotIntent> active_peers_;

  // ROS 2 Interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_raw_sub_;
  rclcpp::Subscription<edge_fleet_msgs::msg::RobotIntent>::SharedPtr peer_intent_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<edge_fleet_msgs::msg::ConflictEvent>::SharedPtr conflict_event_pub_;

  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace edge_fleet_core

#endif  // EDGE_FLEET_CORE_CONFLICT_RESOLVER_NODE_HPP_
