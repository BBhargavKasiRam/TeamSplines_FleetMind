#include <chrono>
#include <memory>
#include <string>
#include <sstream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rmf_task_msgs/msg/api_request.hpp"
#include "edge_fleet_msgs/msg/task_auction.hpp"
#include "std_msgs/msg/bool.hpp"

namespace rmf_edge_ai
{

class RmfEdgeTaskAllocatorNode : public rclcpp::Node
{
public:
  explicit RmfEdgeTaskAllocatorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("rmf_edge_task_allocator_node", options)
  {
    this->declare_parameter<std::string>("robot_name", "tinyRobot1");
    this->declare_parameter<std::string>("fleet_name", "tinyRobot");
    this->declare_parameter<std::string>("pickup_place", "pantry");
    this->declare_parameter<std::string>("dropoff_place", "hardware_2");

    robot_name_ = this->get_parameter("robot_name").as_string();
    fleet_name_ = this->get_parameter("fleet_name").as_string();
    pickup_place_ = this->get_parameter("pickup_place").as_string();
    dropoff_place_ = this->get_parameter("dropoff_place").as_string();

    // Subscriptions
    auction_sub_ = this->create_subscription<edge_fleet_msgs::msg::TaskAuction>(
      "/fleet/task_auctions", rclcpp::QoS(20).reliable(),
      std::bind(&RmfEdgeTaskAllocatorNode::on_auction_msg, this, std::placeholders::_1));

    blockage_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      robot_name_ + "/simulate_aisle_blockage", 10,
      std::bind(&RmfEdgeTaskAllocatorNode::on_blockage_detected, this, std::placeholders::_1));

    // Publishers
    auction_pub_ = this->create_publisher<edge_fleet_msgs::msg::TaskAuction>(
      "/fleet/task_auctions", rclcpp::QoS(20).reliable());

    // Publisher to Open-RMF's task API
    rmf_task_api_pub_ = this->create_publisher<rmf_task_msgs::msg::ApiRequest>(
      "task_api_requests", rclcpp::QoS(10).transient_local());

    RCLCPP_INFO(
      this->get_logger(),
      "[Layer 4: Decentralized Task Allocator] Contract Net Protocol active for [%s].",
      robot_name_.c_str());
  }

private:
  void on_blockage_detected(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      RCLCPP_WARN(
        this->get_logger(),
        "[%s] Blocked aisle prevents task completion! Initiating P2P task auction.",
        robot_name_.c_str());

      edge_fleet_msgs::msg::TaskAuction announce;
      announce.stamp = this->now();
      announce.auction_id = "AUC_RMF_" + robot_name_ + "_" + std::to_string(this->now().nanoseconds());
      announce.task_id = "delivery_" + pickup_place_ + "_to_" + dropoff_place_;
      announce.message_type = edge_fleet_msgs::msg::TaskAuction::MSG_TYPE_ANNOUNCE;
      announce.sender_id = robot_name_;
      announce.reason = "AISLE_BLOCKED_UNREACHABLE";
      auction_pub_->publish(announce);
    }
  }

  void on_auction_msg(const edge_fleet_msgs::msg::TaskAuction::SharedPtr msg)
  {
    // If peer announced a task auction, submit our bid
    if (msg->message_type == edge_fleet_msgs::msg::TaskAuction::MSG_TYPE_ANNOUNCE &&
      msg->sender_id != robot_name_)
    {
      RCLCPP_INFO(
        this->get_logger(),
        "[%s] Received task auction announcement from [%s]. Evaluating local capacity...",
        robot_name_.c_str(), msg->sender_id.c_str());

      edge_fleet_msgs::msg::TaskAuction bid;
      bid.stamp = this->now();
      bid.auction_id = msg->auction_id;
      bid.task_id = msg->task_id;
      bid.message_type = edge_fleet_msgs::msg::TaskAuction::MSG_TYPE_BID;
      bid.sender_id = robot_name_;
      bid.target_robot_id = msg->sender_id;
      bid.bid_cost = 10.5f;  // Estimated cost based on distance and battery
      bid.sender_battery_soc = 92.0f;
      auction_pub_->publish(bid);
    }
    // If our auction received a bid from a peer
    else if (msg->message_type == edge_fleet_msgs::msg::TaskAuction::MSG_TYPE_BID &&
      msg->target_robot_id == robot_name_)
    {
      RCLCPP_INFO(
        this->get_logger(),
        "[%s] Awarding task [%s] to winning peer [%s]! Dispatching directly in Open-RMF.",
        robot_name_.c_str(), msg->task_id.c_str(), msg->sender_id.c_str());

      // Award auction over P2P
      edge_fleet_msgs::msg::TaskAuction award;
      award.stamp = this->now();
      award.auction_id = msg->auction_id;
      award.task_id = msg->task_id;
      award.message_type = edge_fleet_msgs::msg::TaskAuction::MSG_TYPE_AWARD;
      award.sender_id = robot_name_;
      award.target_robot_id = msg->sender_id;
      auction_pub_->publish(award);

      // Construct and dispatch Open-RMF ApiRequest directly for the winning peer robot!
      std::stringstream ss;
      ss << "{\n"
         << "  \"type\": \"dispatch_task_request\",\n"
         << "  \"request\": {\n"
         << "    \"unix_millis_request_time\": 0,\n"
         << "    \"unix_millis_earliest_start_time\": 0,\n"
         << "    \"requester\": \"rmf_edge_ai_p2p\",\n"
         << "    \"category\": \"delivery\",\n"
         << "    \"description\": {\n"
         << "      \"pickup\": {\"place\": \"" << pickup_place_ << "\", \"handler\": \"coke_dispenser\", \"payload\": []},\n"
         << "      \"dropoff\": {\"place\": \"" << dropoff_place_ << "\", \"handler\": \"coke_ingestor\", \"payload\": []}\n"
         << "    }\n"
         << "  }\n"
         << "}\n";

      rmf_task_msgs::msg::ApiRequest api_req;
      api_req.request_id = "p2p_reallocated_" + msg->task_id;
      api_req.json_msg = ss.str();
      rmf_task_api_pub_->publish(api_req);

      RCLCPP_INFO(
        this->get_logger(),
        "[P2P Handoff Succeeded] Task [%s] re-dispatched in Open-RMF to [%s] without central intervention.",
        msg->task_id.c_str(), msg->sender_id.c_str());
    }
  }

  std::string robot_name_;
  std::string fleet_name_;
  std::string pickup_place_;
  std::string dropoff_place_;

  rclcpp::Subscription<edge_fleet_msgs::msg::TaskAuction>::SharedPtr auction_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr blockage_sub_;
  rclcpp::Publisher<edge_fleet_msgs::msg::TaskAuction>::SharedPtr auction_pub_;
  rclcpp::Publisher<rmf_task_msgs::msg::ApiRequest>::SharedPtr rmf_task_api_pub_;
};

}  // namespace rmf_edge_ai

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rmf_edge_ai::RmfEdgeTaskAllocatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
