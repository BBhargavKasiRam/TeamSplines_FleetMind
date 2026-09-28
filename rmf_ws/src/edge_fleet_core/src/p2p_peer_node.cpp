#include "edge_fleet_core/p2p_peer_node.hpp"

#include <cmath>
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace edge_fleet_core
{

P2PPeerNode::P2PPeerNode(const rclcpp::NodeOptions & options)
: Node("p2p_peer_node", options)
{
  this->declare_parameter<std::string>("robot_id", "robot_1");
  this->declare_parameter<std::string>("broadcast_topic", "/fleet/p2p_intents");
  this->declare_parameter<double>("broadcast_rate_hz", 10.0);
  this->declare_parameter<double>("heartbeat_timeout_sec", 2.0);
  this->declare_parameter<double>("battery_drain_rate", 0.02);
  this->declare_parameter<float>("initial_battery_soc", 100.0f);
  this->declare_parameter<int>("priority_level", 5);

  robot_id_ = this->get_parameter("robot_id").as_string();
  broadcast_topic_ = this->get_parameter("broadcast_topic").as_string();
  broadcast_rate_hz_ = this->get_parameter("broadcast_rate_hz").as_double();
  heartbeat_timeout_sec_ = this->get_parameter("heartbeat_timeout_sec").as_double();
  battery_drain_rate_ = this->get_parameter("battery_drain_rate").as_double();
  current_battery_soc_ = this->get_parameter("initial_battery_soc").as_double();
  priority_level_ = static_cast<uint8_t>(this->get_parameter("priority_level").as_int());

  // Subscriptions
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10,
    std::bind(&P2PPeerNode::on_odom_received, this, std::placeholders::_1));

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "plan", 10,
    std::bind(&P2PPeerNode::on_path_received, this, std::placeholders::_1));

  p2p_intent_sub_ = this->create_subscription<edge_fleet_msgs::msg::RobotIntent>(
    broadcast_topic_, rclcpp::QoS(20).reliable(),
    std::bind(&P2PPeerNode::on_peer_intent_received, this, std::placeholders::_1));

  aisle_blocked_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "aisle_blocked", 10,
    std::bind(&P2PPeerNode::on_aisle_blocked, this, std::placeholders::_1));

  assigned_task_sub_ = this->create_subscription<std_msgs::msg::String>(
    "current_assigned_task", 10,
    std::bind(&P2PPeerNode::on_assigned_task, this, std::placeholders::_1));

  // Publishers
  p2p_intent_pub_ = this->create_publisher<edge_fleet_msgs::msg::RobotIntent>(
    broadcast_topic_, rclcpp::QoS(20).reliable());

  battery_pub_ = this->create_publisher<std_msgs::msg::Float32>("battery_soc", 10);

  // Timers
  auto broadcast_period = std::chrono::duration<double>(1.0 / broadcast_rate_hz_);
  broadcast_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(broadcast_period),
    std::bind(&P2PPeerNode::broadcast_timer_callback, this));

  health_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&P2PPeerNode::health_check_callback, this));

  RCLCPP_INFO(
    this->get_logger(),
    "[%s] Edge P2P Decentralized Peer Node started (Mesh Topic: %s @ %.1f Hz)",
    robot_id_.c_str(), broadcast_topic_.c_str(), broadcast_rate_hz_);
}

void P2PPeerNode::on_odom_received(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  has_received_odom_ = true;
  current_pose_.x = msg->pose.pose.position.x;
  current_pose_.y = msg->pose.pose.position.y;

  // Extract yaw
  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  current_pose_.theta = yaw;

  current_velocity_ = msg->twist.twist;

  // Battery drain simulation
  double speed = std::hypot(current_velocity_.linear.x, current_velocity_.linear.y);
  if (speed > 0.05) {
    current_battery_soc_ = std::max(0.0f, current_battery_soc_ - static_cast<float>(battery_drain_rate_ * 0.05));
  }
}

void P2PPeerNode::on_path_received(const nav_msgs::msg::Path::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  predicted_path_.clear();

  // Subsample planned path for compact edge network transmission (up to 15 waypoints)
  size_t step = std::max<size_t>(1, msg->poses.size() / 15);
  for (size_t i = 0; i < msg->poses.size(); i += step) {
    geometry_msgs::msg::Pose2D pt;
    pt.x = msg->poses[i].pose.position.x;
    pt.y = msg->poses[i].pose.position.y;

    tf2::Quaternion q(
      msg->poses[i].pose.orientation.x,
      msg->poses[i].pose.orientation.y,
      msg->poses[i].pose.orientation.z,
      msg->poses[i].pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    pt.theta = yaw;

    predicted_path_.push_back(pt);
  }
}

void P2PPeerNode::on_peer_intent_received(const edge_fleet_msgs::msg::RobotIntent::SharedPtr msg)
{
  // Ignore our own broadcast
  if (msg->robot_id == robot_id_) {
    return;
  }

  // Fleet isolation check: in dual-fleet race mode, only track peers within same fleet
  bool is_edge_fleet = (robot_id_.rfind("edge_", 0) == 0);
  bool is_trad_fleet = (robot_id_.rfind("trad_", 0) == 0);
  if (is_edge_fleet && msg->robot_id.rfind("edge_", 0) != 0) {
    return;
  }
  if (is_trad_fleet && msg->robot_id.rfind("trad_", 0) != 0) {
    return;
  }

  std::lock_guard<std::mutex> lock(peers_mutex_);
  PeerState & state = peers_cache_[msg->robot_id];
  state.intent = *msg;
  state.last_heartbeat = this->now();
  state.is_active = true;
}

void P2PPeerNode::on_aisle_blocked(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  is_blocked_ = msg->data;
}

void P2PPeerNode::on_assigned_task(const std_msgs::msg::String::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  current_task_id_ = msg->data;
}

void P2PPeerNode::broadcast_timer_callback()
{
  edge_fleet_msgs::msg::RobotIntent msg;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!has_received_odom_) {
      return;
    }
    msg.stamp = this->now();
    msg.robot_id = robot_id_;
    msg.current_pose = current_pose_;
    msg.current_velocity = current_velocity_;
    msg.battery_soc = current_battery_soc_;
    msg.predicted_path = predicted_path_;
    msg.current_task_id = current_task_id_;
    msg.priority_level = priority_level_;
    msg.is_yielding = is_yielding_;
    msg.is_blocked = is_blocked_;
  }

  p2p_intent_pub_->publish(msg);

  // Publish battery SoC
  std_msgs::msg::Float32 bat_msg;
  bat_msg.data = msg.battery_soc;
  battery_pub_->publish(bat_msg);
}

void P2PPeerNode::health_check_callback()
{
  std::lock_guard<std::mutex> lock(peers_mutex_);
  auto current_time = this->now();

  for (auto & [id, peer] : peers_cache_) {
    double elapsed = (current_time - peer.last_heartbeat).seconds();
    if (elapsed > heartbeat_timeout_sec_ && peer.is_active) {
      peer.is_active = false;
      RCLCPP_WARN(
        this->get_logger(),
        "[%s] Peer %s heartbeat timed out (elapsed %.2fs). Degrading to local sensing envelope.",
        robot_id_.c_str(), id.c_str(), elapsed);
    }
  }
}

}  // namespace edge_fleet_core

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(edge_fleet_core::P2PPeerNode)

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<edge_fleet_core::P2PPeerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
