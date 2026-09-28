#include "edge_fleet_core/edge_perception_node.hpp"

#include <cmath>

namespace edge_fleet_core
{

EdgePerceptionNode::EdgePerceptionNode(const rclcpp::NodeOptions & options)
: Node("edge_perception_node", options)
{
  this->declare_parameter<std::string>("robot_id", "robot_1");
  this->declare_parameter<double>("forward_angle_range_deg", 35.0);
  this->declare_parameter<double>("blockage_distance_threshold", 1.8);
  this->declare_parameter<int>("min_cluster_points", 5);

  robot_id_ = this->get_parameter("robot_id").as_string();
  forward_angle_range_deg_ = this->get_parameter("forward_angle_range_deg").as_double();
  blockage_distance_threshold_ = this->get_parameter("blockage_distance_threshold").as_double();
  min_cluster_points_ = this->get_parameter("min_cluster_points").as_int();

  // Subscriptions
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", 10,
    std::bind(&EdgePerceptionNode::on_scan, this, std::placeholders::_1));

  manual_trigger_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "simulate_aisle_blockage", 10,
    std::bind(&EdgePerceptionNode::on_manual_blockage_trigger, this, std::placeholders::_1));

  // Publishers
  aisle_blocked_pub_ = this->create_publisher<std_msgs::msg::Bool>("aisle_blocked", 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("perception_markers", 10);

  // Periodic evaluation @ 10Hz
  eval_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&EdgePerceptionNode::evaluate_perception_callback, this));

  RCLCPP_INFO(
    this->get_logger(),
    "[%s] Edge Perception Engine started (Field-of-View: +/-%.1f deg, Blockage Range: %.2fm)",
    robot_id_.c_str(), forward_angle_range_deg_, blockage_distance_threshold_);
}

void EdgePerceptionNode::on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  double angle_limit_rad = forward_angle_range_deg_ * (M_PI / 180.0);
  int blocked_points = 0;

  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    double angle = msg->angle_min + i * msg->angle_increment;
    if (std::abs(angle) <= angle_limit_rad) {
      double r = msg->ranges[i];
      if (r >= msg->range_min && r <= blockage_distance_threshold_) {
        blocked_points++;
      }
    }
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (blocked_points >= min_cluster_points_) {
    continuous_blocked_frames_++;
  } else {
    continuous_blocked_frames_ = std::max(0, continuous_blocked_frames_ - 1);
  }
}

void EdgePerceptionNode::on_manual_blockage_trigger(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  manual_blocked_override_ = msg->data;
  RCLCPP_WARN(
    this->get_logger(),
    "[%s] Manual blockage simulation trigger set to: %s",
    robot_id_.c_str(), manual_blocked_override_ ? "BLOCKED" : "CLEAR");
}

void EdgePerceptionNode::evaluate_perception_callback()
{
  std::lock_guard<std::mutex> lock(mutex_);
  // Blockage confirmed if sustained for at least 5 frames (0.5s) or manual trigger active
  bool blocked = manual_blocked_override_ || (continuous_blocked_frames_ >= 5);

  if (blocked != is_currently_blocked_) {
    is_currently_blocked_ = blocked;
    if (is_currently_blocked_) {
      RCLCPP_WARN(
        this->get_logger(),
        "[%s] HAZARD DETECTED: Aisle is completely blocked ahead! Signaling re-routing engine.",
        robot_id_.c_str());
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "[%s] Aisle cleared.",
        robot_id_.c_str());
    }
  }

  std_msgs::msg::Bool out_msg;
  out_msg.data = is_currently_blocked_;
  aisle_blocked_pub_->publish(out_msg);
}

}  // namespace edge_fleet_core

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(edge_fleet_core::EdgePerceptionNode)

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<edge_fleet_core::EdgePerceptionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
