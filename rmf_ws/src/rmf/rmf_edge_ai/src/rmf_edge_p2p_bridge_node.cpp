#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "rmf_fleet_msgs/msg/fleet_state.hpp"
#include "edge_fleet_msgs/msg/robot_intent.hpp"

namespace rmf_edge_ai
{

class RmfEdgeP2PBridgeNode : public rclcpp::Node
{
public:
  explicit RmfEdgeP2PBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("rmf_edge_p2p_bridge_node", options)
  {
    this->declare_parameter<std::string>("fleet_name", "tinyRobot");
    fleet_name_ = this->get_parameter("fleet_name").as_string();

    // Subscribe to Open-RMF's fleet state topic
    fleet_state_sub_ = this->create_subscription<rmf_fleet_msgs::msg::FleetState>(
      "fleet_states", rclcpp::QoS(10),
      std::bind(&RmfEdgeP2PBridgeNode::on_fleet_state, this, std::placeholders::_1));

    // Publisher for decentralized P2P intent mesh
    p2p_intent_pub_ = this->create_publisher<edge_fleet_msgs::msg::RobotIntent>(
      "/fleet/p2p_intents", rclcpp::QoS(20).reliable());

    RCLCPP_INFO(
      this->get_logger(),
      "[Layer 2: Decentralized P2P Mesh] Bridging Open-RMF fleet [%s] to /fleet/p2p_intents.",
      fleet_name_.c_str());
  }

private:
  void on_fleet_state(const rmf_fleet_msgs::msg::FleetState::SharedPtr msg)
  {
    if (msg->name != fleet_name_) {
      return;
    }

    for (const auto & robot : msg->robots) {
      edge_fleet_msgs::msg::RobotIntent intent;
      intent.stamp = this->now();
      intent.robot_id = robot.name;
      intent.current_pose.x = robot.location.x;
      intent.current_pose.y = robot.location.y;
      intent.current_pose.theta = robot.location.yaw;
      intent.battery_soc = robot.battery_percent;
      intent.current_task_id = robot.task_id;
      intent.priority_level = (robot.name == "tinyRobot1") ? 7 : 5;

      // Extract waypoints from Open-RMF robot path
      for (const auto & loc : robot.path) {
        geometry_msgs::msg::Pose2D pt;
        pt.x = loc.x;
        pt.y = loc.y;
        pt.theta = loc.yaw;
        intent.predicted_path.push_back(pt);
      }

      p2p_intent_pub_->publish(intent);
    }
  }

  std::string fleet_name_;
  rclcpp::Subscription<rmf_fleet_msgs::msg::FleetState>::SharedPtr fleet_state_sub_;
  rclcpp::Publisher<edge_fleet_msgs::msg::RobotIntent>::SharedPtr p2p_intent_pub_;
};

}  // namespace rmf_edge_ai

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rmf_edge_ai::RmfEdgeP2PBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
