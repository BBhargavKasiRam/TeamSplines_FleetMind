#include "edge_fleet_core/task_allocator_node.hpp"

#include <cmath>
#include <algorithm>

namespace edge_fleet_core
{

TaskAllocatorNode::TaskAllocatorNode(const rclcpp::NodeOptions & options)
: Node("task_allocator_node", options)
{
  this->declare_parameter<std::string>("robot_id", "robot_1");
  robot_id_ = this->get_parameter("robot_id").as_string();

  // Subscriptions
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10,
    std::bind(&TaskAllocatorNode::on_odom, this, std::placeholders::_1));

  auction_sub_ = this->create_subscription<edge_fleet_msgs::msg::TaskAuction>(
    "/fleet/task_auctions", rclcpp::QoS(20).reliable(),
    std::bind(&TaskAllocatorNode::on_auction_msg, this, std::placeholders::_1));

  aisle_blocked_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "aisle_blocked", 10,
    std::bind(&TaskAllocatorNode::on_aisle_blocked_signal, this, std::placeholders::_1));

  // Publishers
  auction_pub_ = this->create_publisher<edge_fleet_msgs::msg::TaskAuction>(
    "/fleet/task_auctions", rclcpp::QoS(20).reliable());

  assigned_task_pub_ = this->create_publisher<std_msgs::msg::String>(
    "current_assigned_task", 10);

  // Timer to close auction after bidding window (e.g. 600 ms)
  auction_eval_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&TaskAllocatorNode::auction_timer_callback, this));

  RCLCPP_INFO(
    this->get_logger(),
    "[%s] Decentralized Task Allocator (Contract Net Protocol) initialized.",
    robot_id_.c_str());
}

void TaskAllocatorNode::on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  current_pos_.x = msg->pose.pose.position.x;
  current_pos_.y = msg->pose.pose.position.y;
}

void TaskAllocatorNode::on_aisle_blocked_signal(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    RCLCPP_WARN(
      this->get_logger(),
      "[%s] Aisle Blockage detected! Triggering P2P Task Re-allocation via Contract Net Protocol.",
      robot_id_.c_str());

    geometry_msgs::msg::Point pickup, dropoff;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      pickup.x = current_pos_.x + 5.0;
      pickup.y = current_pos_.y + 2.0;
      dropoff.x = 0.0;
      dropoff.y = 0.0;
    }
    trigger_task_auction("TASK_REALLOC_" + robot_id_, pickup, dropoff, "AISLE_BLOCKED");
  }
}

void TaskAllocatorNode::trigger_task_auction(
  const std::string & task_id,
  const geometry_msgs::msg::Point & pickup,
  const geometry_msgs::msg::Point & dropoff,
  const std::string & reason)
{
  std::lock_guard<std::mutex> lock(mutex_);
  is_auction_active_ = true;
  active_auction_id_ = "AUC_" + robot_id_ + "_" + std::to_string(this->now().nanoseconds());
  active_task_id_ = task_id;
  active_pickup_ = pickup;
  active_dropoff_ = dropoff;
  collected_bids_.clear();
  auction_start_time_ = this->now();

  edge_fleet_msgs::msg::TaskAuction announce;
  announce.stamp = this->now();
  announce.auction_id = active_auction_id_;
  announce.task_id = active_task_id_;
  announce.message_type = edge_fleet_msgs::msg::TaskAuction::MSG_TYPE_ANNOUNCE;
  announce.sender_id = robot_id_;
  announce.pickup_location = active_pickup_;
  announce.dropoff_location = active_dropoff_;
  announce.reason = reason;

  auction_pub_->publish(announce);
  RCLCPP_INFO(
    this->get_logger(),
    "[%s] Broadcasted AUCTION_ANNOUNCE for Task '%s' due to [%s]",
    robot_id_.c_str(), task_id.c_str(), reason.c_str());
}

void TaskAllocatorNode::on_auction_msg(const edge_fleet_msgs::msg::TaskAuction::SharedPtr msg)
{
  // 1. If someone else announced an auction, evaluate our cost and submit a BID
  if (msg->message_type == edge_fleet_msgs::msg::TaskAuction::MSG_TYPE_ANNOUNCE &&
    msg->sender_id != robot_id_)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    double dist_to_pickup = std::hypot(
      current_pos_.x - msg->pickup_location.x,
      current_pos_.y - msg->pickup_location.y);

    // Bid Cost: weighted distance + penalty for lower battery
    float battery_penalty = (100.0f - current_battery_soc_) * 0.1f;
    float total_cost = static_cast<float>(dist_to_pickup) + battery_penalty;

    edge_fleet_msgs::msg::TaskAuction bid;
    bid.stamp = this->now();
    bid.auction_id = msg->auction_id;
    bid.task_id = msg->task_id;
    bid.message_type = edge_fleet_msgs::msg::TaskAuction::MSG_TYPE_BID;
    bid.sender_id = robot_id_;
    bid.target_robot_id = msg->sender_id;
    bid.bid_cost = total_cost;
    bid.sender_battery_soc = current_battery_soc_;

    auction_pub_->publish(bid);
    RCLCPP_INFO(
      this->get_logger(),
      "[%s] Submitted BID: cost=%.2f for Task '%s' (dist=%.2fm, soc=%.1f%%)",
      robot_id_.c_str(), total_cost, msg->task_id.c_str(), dist_to_pickup, current_battery_soc_);
  }
  // 2. If we are running the auction and received a BID
  else if (msg->message_type == edge_fleet_msgs::msg::TaskAuction::MSG_TYPE_BID &&
    msg->target_robot_id == robot_id_)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_auction_active_ && msg->auction_id == active_auction_id_) {
      BidInfo b;
      b.bidder_id = msg->sender_id;
      b.cost = msg->bid_cost;
      b.battery_soc = msg->sender_battery_soc;
      collected_bids_.push_back(b);
    }
  }
  // 3. If an auction was awarded to US
  else if (msg->message_type == edge_fleet_msgs::msg::TaskAuction::MSG_TYPE_AWARD &&
    msg->target_robot_id == robot_id_)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "[%s] Task '%s' AWARDED to me! Accepting and updating local task planner.",
      robot_id_.c_str(), msg->task_id.c_str());

    edge_fleet_msgs::msg::TaskAuction ack;
    ack.stamp = this->now();
    ack.auction_id = msg->auction_id;
    ack.task_id = msg->task_id;
    ack.message_type = edge_fleet_msgs::msg::TaskAuction::MSG_TYPE_ACK;
    ack.sender_id = robot_id_;
    ack.target_robot_id = msg->sender_id;
    auction_pub_->publish(ack);

    std_msgs::msg::String task_msg;
    task_msg.data = msg->task_id;
    assigned_task_pub_->publish(task_msg);
  }
}

void TaskAllocatorNode::auction_timer_callback()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!is_auction_active_) {
    return;
  }

  double elapsed = (this->now() - auction_start_time_).seconds();
  // Close auction after 0.5s window
  if (elapsed >= 0.5) {
    is_auction_active_ = false;

    if (collected_bids_.empty()) {
      RCLCPP_WARN(
        this->get_logger(),
        "[%s] Auction %s finished with 0 bids. Retaining task locally.",
        robot_id_.c_str(), active_auction_id_.c_str());
      return;
    }

    // Pick best bidder (lowest cost)
    auto best_bid = std::min_element(
      collected_bids_.begin(), collected_bids_.end(),
      [](const BidInfo & a, const BidInfo & b) { return a.cost < b.cost; });

    RCLCPP_INFO(
      this->get_logger(),
      "[%s] Auction WON by '%s' with cost %.2f! Broadcasting AWARD.",
      robot_id_.c_str(), best_bid->bidder_id.c_str(), best_bid->cost);

    edge_fleet_msgs::msg::TaskAuction award;
    award.stamp = this->now();
    award.auction_id = active_auction_id_;
    award.task_id = active_task_id_;
    award.message_type = edge_fleet_msgs::msg::TaskAuction::MSG_TYPE_AWARD;
    award.sender_id = robot_id_;
    award.target_robot_id = best_bid->bidder_id;
    award.pickup_location = active_pickup_;
    award.dropoff_location = active_dropoff_;
    auction_pub_->publish(award);
  }
}

}  // namespace edge_fleet_core

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(edge_fleet_core::TaskAllocatorNode)

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<edge_fleet_core::TaskAllocatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
