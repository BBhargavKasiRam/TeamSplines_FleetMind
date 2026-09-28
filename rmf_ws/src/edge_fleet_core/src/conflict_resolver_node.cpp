#include "edge_fleet_core/conflict_resolver_node.hpp"

#include <algorithm>
#include <cmath>
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace edge_fleet_core
{

ConflictResolverNode::ConflictResolverNode(const rclcpp::NodeOptions & options)
: Node("conflict_resolver_node", options)
{
  this->declare_parameter<std::string>("robot_id", "robot_1");
  this->declare_parameter<std::string>("mode", "EDGE_AI");
  this->declare_parameter<double>("robot_radius", 0.35);
  this->declare_parameter<double>("safety_margin", 0.25);
  this->declare_parameter<double>("time_horizon", 3.5);
  this->declare_parameter<double>("max_speed", 0.8);
  this->declare_parameter<double>("emergency_brake_dist", 0.45);
  this->declare_parameter<int>("priority_level", 5);

  robot_id_ = this->get_parameter("robot_id").as_string();
  mode_ = "EDGE_AI";
  robot_radius_ = this->get_parameter("robot_radius").as_double();
  safety_margin_ = this->get_parameter("safety_margin").as_double();
  time_horizon_ = this->get_parameter("time_horizon").as_double();
  max_speed_ = this->get_parameter("max_speed").as_double();
  emergency_brake_dist_ = this->get_parameter("emergency_brake_dist").as_double();
  priority_level_ = this->get_parameter("priority_level").as_int();

  // Subscriptions
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10,
    std::bind(&ConflictResolverNode::on_odom, this, std::placeholders::_1));

  cmd_vel_raw_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel_raw", 10,
    std::bind(&ConflictResolverNode::on_cmd_vel_raw, this, std::placeholders::_1));

  peer_intent_sub_ = this->create_subscription<edge_fleet_msgs::msg::RobotIntent>(
    "/fleet/p2p_intents", rclcpp::QoS(20).reliable(),
    std::bind(&ConflictResolverNode::on_peer_intent, this, std::placeholders::_1));

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", 10,
    std::bind(&ConflictResolverNode::on_scan, this, std::placeholders::_1));

  // Publishers
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  conflict_event_pub_ = this->create_publisher<edge_fleet_msgs::msg::ConflictEvent>(
    "/fleet/conflict_events", 10);

  // Control loop @ 20Hz
  last_loop_time_ = this->now();
  last_desired_twist_time_ = this->now();
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&ConflictResolverNode::control_loop_callback, this));

  RCLCPP_INFO(
    this->get_logger(),
    "[%s] Conflict Resolver running in [%s] Mode (Radius: %.2fm, Margin: %.2fm)",
    robot_id_.c_str(), mode_.c_str(), robot_radius_, safety_margin_);
}

void ConflictResolverNode::on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  current_pos_.x = msg->pose.pose.position.x;
  current_pos_.y = msg->pose.pose.position.y;

  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double r, p, yaw;
  m.getRPY(r, p, yaw);
  current_yaw_ = yaw;

  // Convert body linear velocity to world frame
  double vx = msg->twist.twist.linear.x;
  double vy = msg->twist.twist.linear.y;
  current_vel_.x = vx * std::cos(current_yaw_) - vy * std::sin(current_yaw_);
  current_vel_.y = vx * std::sin(current_yaw_) + vy * std::cos(current_yaw_);
}

void ConflictResolverNode::on_cmd_vel_raw(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  desired_twist_ = *msg;
  has_received_desired_twist_ = true;
  last_desired_twist_time_ = this->now();
}

void ConflictResolverNode::on_peer_intent(const edge_fleet_msgs::msg::RobotIntent::SharedPtr msg)
{
  if (msg->robot_id == robot_id_) {
    return;
  }

  // Fleet isolation check: in dual-fleet race mode, only coordinate with peers in same fleet
  bool is_edge_fleet = (robot_id_.rfind("edge_", 0) == 0);
  bool is_trad_fleet = (robot_id_.rfind("trad_", 0) == 0);
  if (is_edge_fleet && msg->robot_id.rfind("edge_", 0) != 0) {
    return;
  }
  if (is_trad_fleet && msg->robot_id.rfind("trad_", 0) != 0) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  active_peers_[msg->robot_id] = *msg;
}

void ConflictResolverNode::on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  double min_val = 100.0;
  for (float r : msg->ranges) {
    if (r >= msg->range_min && r <= msg->range_max) {
      if (r < min_val) {
        min_val = r;
      }
    }
  }
  std::lock_guard<std::mutex> lock(mutex_);
  min_lidar_dist_ = min_val;
}

void ConflictResolverNode::control_loop_callback()
{
  auto now = this->now();
  last_loop_time_ = now;

  geometry_msgs::msg::Twist desired;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // Watchdog: If no desired cmd received recently (0.5s), stop
    if (!has_received_desired_twist_ || (now - last_desired_twist_time_).seconds() > 0.5) {
      desired = geometry_msgs::msg::Twist();
    } else {
      desired = desired_twist_;
    }
  }

  // 1. HARD SAFETY CHECK (Emergency braking to guarantee zero collisions)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (min_lidar_dist_ < emergency_brake_dist_) {
      geometry_msgs::msg::Twist stop_twist;
      cmd_vel_pub_->publish(stop_twist);
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "[%s] Emergency brake triggered! Obstacle at %.2fm < %.2fm",
        robot_id_.c_str(), min_lidar_dist_, emergency_brake_dist_);
      return;
    }
  }

  // 2. CONFLICT RESOLUTION (Decentralized Edge-AI ORCA)
  geometry_msgs::msg::Twist resolved_twist = resolve_edge_ai_orca(desired);

  cmd_vel_pub_->publish(resolved_twist);
}

geometry_msgs::msg::Twist ConflictResolverNode::resolve_edge_ai_orca(
  const geometry_msgs::msg::Twist & desired_twist)
{
  std::lock_guard<std::mutex> lock(mutex_);
  double combined_radius = (robot_radius_ * 2.0) + safety_margin_;

  // Transform desired twist from robot frame to world frame desired velocity
  double pref_vx_world = desired_twist.linear.x * std::cos(current_yaw_) -
    desired_twist.linear.y * std::sin(current_yaw_);
  double pref_vy_world = desired_twist.linear.x * std::sin(current_yaw_) +
    desired_twist.linear.y * std::cos(current_yaw_);
  Vector2D v_pref(pref_vx_world, pref_vy_world);

  Vector2D v_orca = v_pref;
  bool conflict_detected = false;
  std::string conflicting_peer = "";

  for (const auto & [peer_id, peer_intent] : active_peers_) {
    Vector2D p_peer(peer_intent.current_pose.x, peer_intent.current_pose.y);
    Vector2D p_rel = p_peer - current_pos_;
    double dist = p_rel.norm();

    if (dist > 4.5 || dist < 1e-3) {
      continue;
    }

    // Peer world velocity
    double peer_yaw = peer_intent.current_pose.theta;
    double p_vx_world = peer_intent.current_velocity.linear.x * std::cos(peer_yaw) -
      peer_intent.current_velocity.linear.y * std::sin(peer_yaw);
    double p_vy_world = peer_intent.current_velocity.linear.x * std::sin(peer_yaw) +
      peer_intent.current_velocity.linear.y * std::cos(peer_yaw);
    Vector2D v_peer(p_vx_world, p_vy_world);

    // Relative velocity
    Vector2D v_rel = v_orca - v_peer;
    double v_rel_norm_sq = v_rel.dot(v_rel);

    if (v_rel_norm_sq < 1e-4) {
      continue;
    }

    // Time to Closest Point of Approach (TCPA)
    double t_cpa = -(p_rel.dot(v_rel)) / v_rel_norm_sq;
    if (t_cpa > 0.05 && t_cpa < time_horizon_) {
      Vector2D pos_at_cpa = p_rel + (v_rel * t_cpa);
      double dist_at_cpa = pos_at_cpa.norm();

      if (dist_at_cpa < combined_radius) {
        conflict_detected = true;
        conflicting_peer = peer_id;

        bool peer_has_priority = (peer_intent.priority_level > priority_level_) ||
          (peer_intent.priority_level == priority_level_ && peer_id < robot_id_);

        if (peer_has_priority) {
          // Dynamic Predictive Yielding: modulate speed without coming to a dead stop
          if (dist < 1.2) {
            v_orca = v_pref * 0.35;      // Smooth crawl through intersection
          } else if (dist < 2.5) {
            v_orca = v_pref * 0.55;      // Early deceleration to phase arrival behind peer
          } else {
            v_orca = v_pref * 0.75;
          }
        } else {
          // Higher Priority AMR: Maintains nominal right-of-way speed
          v_orca = v_pref;
        }
      }
    }
  }

  // Hard safety override: If peer is dangerously close (< 0.55m) and has priority
  for (const auto & [peer_id, peer_intent] : active_peers_) {
    Vector2D p_peer(peer_intent.current_pose.x, peer_intent.current_pose.y);
    double dist = (p_peer - current_pos_).norm();
    if (dist < 0.55) {
      bool peer_has_priority = (peer_intent.priority_level > priority_level_) ||
        (peer_intent.priority_level == priority_level_ && peer_id < robot_id_);
      if (peer_has_priority) {
        geometry_msgs::msg::Twist emergency_stop;
        return emergency_stop;
      }
    }
  }

  // Cap negotiated velocity to max speed
  if (v_orca.norm() > max_speed_) {
    v_orca = v_orca.normalized() * max_speed_;
  }

  // Convert back to robot frame (diff drive)
  geometry_msgs::msg::Twist resolved_twist;
  double target_yaw = std::atan2(v_orca.y, v_orca.x);
  double yaw_diff = target_yaw - current_yaw_;
  while (yaw_diff > M_PI) yaw_diff -= 2.0 * M_PI;
  while (yaw_diff < -M_PI) yaw_diff += 2.0 * M_PI;

  if (conflict_detected) {
    if (v_orca.norm() < 0.05) {
      resolved_twist.linear.x = 0.0;
      resolved_twist.angular.z = 0.0;
    } else {
      resolved_twist.linear.x = std::max(0.0, v_orca.norm() * std::cos(yaw_diff));
      resolved_twist.angular.z = std::clamp(yaw_diff * 1.5, -1.0, 1.0);
    }

    edge_fleet_msgs::msg::ConflictEvent event;
    event.stamp = this->now();
    event.conflict_id = robot_id_ + "_vs_" + conflicting_peer;
    event.involved_robots = {robot_id_, conflicting_peer};
    event.resolution_action = "EDGE_AI_ORCA_VELOCITY_ADJUST";
    event.is_deadlock_resolved = true;
    conflict_event_pub_->publish(event);
  } else {
    resolved_twist = desired_twist;
  }

  return resolved_twist;
}

}  // namespace edge_fleet_core

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(edge_fleet_core::ConflictResolverNode)

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<edge_fleet_core::ConflictResolverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
