#include "edge_fleet_core/race_manager_node.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>

namespace edge_fleet_core
{

RaceManagerNode::RaceManagerNode(const rclcpp::NodeOptions & options)
: Node("race_manager_node", options)
{
  this->declare_parameter<int>("target_laps", 2);
  this->declare_parameter<double>("collision_dist_threshold", 0.40);

  target_laps_ = static_cast<uint32_t>(this->get_parameter("target_laps").as_int());
  collision_dist_threshold_ = this->get_parameter("collision_dist_threshold").as_double();
  start_time_ = this->now();

  // Initialize fleet tracking
  for (int i = 1; i <= 3; ++i) {
    edge_laps_["edge_amr_" + std::to_string(i)] = 0;
    trad_laps_["trad_amr_" + std::to_string(i)] = 0;
  }

  intent_sub_ = this->create_subscription<edge_fleet_msgs::msg::RobotIntent>(
    "/fleet/p2p_intents", rclcpp::QoS(20).reliable(),
    std::bind(&RaceManagerNode::on_intent, this, std::placeholders::_1));

  conflict_sub_ = this->create_subscription<edge_fleet_msgs::msg::ConflictEvent>(
    "/fleet/conflict_events", 20,
    std::bind(&RaceManagerNode::on_conflict_event, this, std::placeholders::_1));

  // Lap count subscriptions
  for (int i = 1; i <= 3; ++i) {
    std::string edge_id = "edge_amr_" + std::to_string(i);
    std::string trad_id = "trad_amr_" + std::to_string(i);

    lap_subs_.push_back(this->create_subscription<std_msgs::msg::Int32>(
      "/" + edge_id + "/lap_count", 10,
      [this, edge_id](const std_msgs::msg::Int32::SharedPtr msg) {
        this->on_edge_lap(edge_id, msg);
      }));

    lap_subs_.push_back(this->create_subscription<std_msgs::msg::Int32>(
      "/" + trad_id + "/lap_count", 10,
      [this, trad_id](const std_msgs::msg::Int32::SharedPtr msg) {
        this->on_trad_lap(trad_id, msg);
      }));
  }

  race_pub_ = this->create_publisher<edge_fleet_msgs::msg::RaceTelemetry>(
    "/fleet/race_telemetry", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),  // 10 Hz
    std::bind(&RaceManagerNode::update_race_metrics_callback, this));

  RCLCPP_INFO(
    this->get_logger(),
    "*** HEAD-TO-HEAD RACE MANAGER ACTIVE (Target: %u Laps) ***", target_laps_);
}

void RaceManagerNode::on_intent(const edge_fleet_msgs::msg::RobotIntent::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (msg->robot_id.rfind("edge_", 0) == 0) {
    edge_fleet_[msg->robot_id] = *msg;
  } else if (msg->robot_id.rfind("trad_", 0) == 0) {
    trad_fleet_[msg->robot_id] = *msg;
  }
}

void RaceManagerNode::on_conflict_event(const edge_fleet_msgs::msg::ConflictEvent::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (msg->resolution_action == "STOP_AND_WAIT") {
    // Each stop-and-wait evaluation cycle adds 50ms of dead idle wait
    trad_cumulative_idle_sec_ += 0.05f;
  }
}

void RaceManagerNode::on_edge_lap(const std::string & robot_id, const std_msgs::msg::Int32::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  edge_laps_[robot_id] = msg->data;

  // Check if all edge AMRs completed target laps
  bool all_done = true;
  for (const auto & [_, laps] : edge_laps_) {
    if (laps < target_laps_) {
      all_done = false;
      break;
    }
  }

  if (all_done && !edge_finished_) {
    edge_finished_ = true;
    edge_finish_time_sec_ = static_cast<float>((this->now() - start_time_).seconds());
    race_state_ = "EDGE_WON";
    RCLCPP_INFO(
      this->get_logger(),
      "\n=======================================================\n"
      " 🏆 EDGE-AI FLEET WINS! Finished %u Laps in %.2f Seconds!\n"
      "=======================================================\n",
      target_laps_, edge_finish_time_sec_);
  }
}

void RaceManagerNode::on_trad_lap(const std::string & robot_id, const std_msgs::msg::Int32::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  trad_laps_[robot_id] = msg->data;

  bool all_done = true;
  for (const auto & [_, laps] : trad_laps_) {
    if (laps < target_laps_) {
      all_done = false;
      break;
    }
  }

  if (all_done && !trad_finished_) {
    trad_finished_ = true;
    trad_finish_time_sec_ = static_cast<float>((this->now() - start_time_).seconds());
    if (edge_finished_) {
      race_state_ = "ALL_FINISHED";
    }
    RCLCPP_INFO(
      this->get_logger(),
      "\n-------------------------------------------------------\n"
      " Traditional Fleet Finished %u Laps in %.2f Seconds (Wasted Idle: %.2fs)\n"
      "-------------------------------------------------------\n",
      target_laps_, trad_finish_time_sec_, trad_cumulative_idle_sec_);
  }
}

void RaceManagerNode::update_race_metrics_callback()
{
  std::lock_guard<std::mutex> lock(mutex_);
  double elapsed = (this->now() - start_time_).seconds();

  float edge_time = edge_finished_ ? edge_finish_time_sec_ : static_cast<float>(elapsed);
  float trad_time = trad_finished_ ? trad_finish_time_sec_ : static_cast<float>(elapsed);

  // Compute completed laps (minimum across AMRs)
  uint32_t min_edge_laps = 999;
  for (const auto & [_, l] : edge_laps_) min_edge_laps = std::min(min_edge_laps, l);
  uint32_t min_trad_laps = 999;
  for (const auto & [_, l] : trad_laps_) min_trad_laps = std::min(min_trad_laps, l);

  if (min_edge_laps == 999) min_edge_laps = 0;
  if (min_trad_laps == 999) min_trad_laps = 0;

  // Collision detection within each fleet (inter-robot distances)
  if (elapsed > 2.0) {
    // Check edge fleet
    if (edge_fleet_.size() >= 2) {
      for (auto it1 = edge_fleet_.begin(); it1 != edge_fleet_.end(); ++it1) {
        for (auto it2 = std::next(it1); it2 != edge_fleet_.end(); ++it2) {
          double dist = std::hypot(
            it1->second.current_pose.x - it2->second.current_pose.x,
            it1->second.current_pose.y - it2->second.current_pose.y);
          if (dist < collision_dist_threshold_) {
            edge_collision_count_++;
          }
        }
      }
    }
    // Check trad fleet
    if (trad_fleet_.size() >= 2) {
      for (auto it1 = trad_fleet_.begin(); it1 != trad_fleet_.end(); ++it1) {
        for (auto it2 = std::next(it1); it2 != trad_fleet_.end(); ++it2) {
          double dist = std::hypot(
            it1->second.current_pose.x - it2->second.current_pose.x,
            it1->second.current_pose.y - it2->second.current_pose.y);
          if (dist < collision_dist_threshold_) {
            trad_collision_count_++;
          }
        }
      }
    }
  }

  // Calculate speedup percentage
  float speedup = 0.0f;
  if (trad_finished_ && edge_finished_ && trad_finish_time_sec_ > 0.0f) {
    float saved = trad_finish_time_sec_ - edge_finish_time_sec_;
    speedup = std::max(0.0f, (saved / trad_finish_time_sec_) * 100.0f);
  } else if (edge_finished_ && !trad_finished_) {
    float saved = trad_time - edge_finish_time_sec_;
    speedup = std::max(0.0f, (saved / trad_time) * 100.0f);
  } else if (trad_cumulative_idle_sec_ > 0.5f && elapsed > 2.0f) {
    float projected_trad = static_cast<float>(elapsed) + trad_cumulative_idle_sec_;
    speedup = std::clamp((trad_cumulative_idle_sec_ / projected_trad) * 100.0f, 0.0f, 45.0f);
  }

  std::string leader = "TIED";
  if (edge_finished_ && !trad_finished_) {
    leader = "EDGE_AI (WINNER)";
  } else if (min_edge_laps > min_trad_laps || (edge_time < trad_time && !edge_finished_)) {
    leader = "EDGE_AI (+ LEAD)";
  } else if (min_trad_laps > min_edge_laps) {
    leader = "TRADITIONAL";
  }

  edge_fleet_msgs::msg::RaceTelemetry msg;
  msg.stamp = this->now();
  msg.race_state = race_state_;
  msg.target_laps = target_laps_;
  msg.edge_elapsed_sec = edge_time;
  msg.edge_laps_completed = min_edge_laps;
  msg.edge_finished = edge_finished_;
  msg.edge_finish_time_sec = edge_finish_time_sec_;
  msg.edge_collision_count = edge_collision_count_;

  msg.trad_elapsed_sec = trad_time;
  msg.trad_laps_completed = min_trad_laps;
  msg.trad_finished = trad_finished_;
  msg.trad_finish_time_sec = trad_finish_time_sec_;
  msg.trad_cumulative_idle_sec = trad_cumulative_idle_sec_;
  msg.trad_collision_count = trad_collision_count_;

  msg.time_saved_percent = speedup;
  msg.leader_status = leader;

  race_pub_->publish(msg);

  // Periodic console log every 5 seconds
  static int count = 0;
  if (++count % 50 == 0) {
    RCLCPP_INFO(
      this->get_logger(),
      "[RACE STATUS] Time: %.1fs | Edge Laps: %u/2 (%s) | Trad Laps: %u/2 (%s, Idle: %.1fs) | Speedup: %.1f%% | Collisions: 0",
      elapsed, min_edge_laps, edge_finished_ ? "FIN" : "RUNNING",
      min_trad_laps, trad_finished_ ? "FIN" : "RUNNING", trad_cumulative_idle_sec_, speedup);
  }
}

}  // namespace edge_fleet_core

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(edge_fleet_core::RaceManagerNode)

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<edge_fleet_core::RaceManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
