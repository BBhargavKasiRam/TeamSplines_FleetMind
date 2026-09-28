#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "rmf_fleet_msgs/msg/lane_request.hpp"

namespace rmf_edge_ai
{

class RmfEdgePerceptionNode : public rclcpp::Node
{
public:
  explicit RmfEdgePerceptionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("rmf_edge_perception_node", options)
  {
    this->declare_parameter<std::string>("fleet_name", "tinyRobot");
    this->declare_parameter<std::string>("robot_name", "tinyRobot1");
    this->declare_parameter<int64_t>("default_blocked_lane_id", 12);
    this->declare_parameter<double>("obstacle_dist_threshold", 1.8);

    fleet_name_ = this->get_parameter("fleet_name").as_string();
    robot_name_ = this->get_parameter("robot_name").as_string();
    default_blocked_lane_id_ = static_cast<uint64_t>(this->get_parameter("default_blocked_lane_id").as_int());
    obstacle_dist_threshold_ = this->get_parameter("obstacle_dist_threshold").as_double();

    // Subscriptions
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      robot_name_ + "/scan", 10,
      std::bind(&RmfEdgePerceptionNode::on_scan, this, std::placeholders::_1));

    blockage_trigger_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      robot_name_ + "/simulate_aisle_blockage", 10,
      std::bind(&RmfEdgePerceptionNode::on_blockage_trigger, this, std::placeholders::_1));

    // Publisher to Open-RMF's native lane closure topic
    lane_request_pub_ = this->create_publisher<rmf_fleet_msgs::msg::LaneRequest>(
      "lane_closure_requests", rclcpp::QoS(10).reliable());

    RCLCPP_INFO(
      this->get_logger(),
      "[Layer 1: Edge Perception] Initialized for fleet [%s], robot [%s]. Monitoring aisle hazards.",
      fleet_name_.c_str(), robot_name_.c_str());
  }

private:
  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    double min_dist = 100.0;
    // Check forward 60-degree cone
    size_t mid_idx = msg->ranges.size() / 2;
    size_t span = msg->ranges.size() / 6;
    size_t start = (mid_idx > span) ? (mid_idx - span) : 0;
    size_t end = std::min(msg->ranges.size(), mid_idx + span);

    for (size_t i = start; i < end; ++i) {
      if (msg->ranges[i] >= msg->range_min && msg->ranges[i] <= msg->range_max) {
        if (msg->ranges[i] < min_dist) {
          min_dist = msg->ranges[i];
        }
      }
    }

    if (min_dist < obstacle_dist_threshold_ && !is_lane_closed_) {
      RCLCPP_WARN(
        this->get_logger(),
        "[%s] Laser detected obstacle at %.2fm! Closing lane %lu dynamically in Open-RMF.",
        robot_name_.c_str(), min_dist, default_blocked_lane_id_);
      close_lane(default_blocked_lane_id_);
    }
  }

  void on_blockage_trigger(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data && !is_lane_closed_) {
      RCLCPP_WARN(
        this->get_logger(),
        "[%s] Blockage trigger received! Closing lane %lu in Open-RMF nav graph.",
        robot_name_.c_str(), default_blocked_lane_id_);
      close_lane(default_blocked_lane_id_);
    } else if (!msg->data && is_lane_closed_) {
      RCLCPP_INFO(
        this->get_logger(),
        "[%s] Aisle cleared! Reopening lane %lu in Open-RMF.",
        robot_name_.c_str(), default_blocked_lane_id_);
      open_lane(default_blocked_lane_id_);
    }
  }

  void close_lane(uint64_t lane_id)
  {
    is_lane_closed_ = true;
    rmf_fleet_msgs::msg::LaneRequest req;
    req.fleet_name = fleet_name_;
    req.close_lanes.push_back(lane_id);
    lane_request_pub_->publish(req);
  }

  void open_lane(uint64_t lane_id)
  {
    is_lane_closed_ = false;
    rmf_fleet_msgs::msg::LaneRequest req;
    req.fleet_name = fleet_name_;
    req.open_lanes.push_back(lane_id);
    lane_request_pub_->publish(req);
  }

  std::string fleet_name_;
  std::string robot_name_;
  uint64_t default_blocked_lane_id_{12};
  double obstacle_dist_threshold_{1.8};
  bool is_lane_closed_{false};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr blockage_trigger_sub_;
  rclcpp::Publisher<rmf_fleet_msgs::msg::LaneRequest>::SharedPtr lane_request_pub_;
};

}  // namespace rmf_edge_ai

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rmf_edge_ai::RmfEdgePerceptionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
