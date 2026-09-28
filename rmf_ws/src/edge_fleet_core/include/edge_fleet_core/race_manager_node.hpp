#ifndef EDGE_FLEET_CORE_RACE_MANAGER_NODE_HPP_
#define EDGE_FLEET_CORE_RACE_MANAGER_NODE_HPP_

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "edge_fleet_msgs/msg/robot_intent.hpp"
#include "edge_fleet_msgs/msg/conflict_event.hpp"
#include "edge_fleet_msgs/msg/race_telemetry.hpp"

namespace edge_fleet_core
{

class RaceManagerNode : public rclcpp::Node
{
public:
  explicit RaceManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_intent(const edge_fleet_msgs::msg::RobotIntent::SharedPtr msg);
  void on_conflict_event(const edge_fleet_msgs::msg::ConflictEvent::SharedPtr msg);
  void on_edge_lap(const std::string & robot_id, const std_msgs::msg::Int32::SharedPtr msg);
  void on_trad_lap(const std::string & robot_id, const std_msgs::msg::Int32::SharedPtr msg);
  void update_race_metrics_callback();

  // Parameters
  uint32_t target_laps_{2};
  double collision_dist_threshold_{0.40};

  // State
  rclcpp::Time start_time_;
  std::mutex mutex_;
  std::string race_state_{"RUNNING"};

  std::map<std::string, edge_fleet_msgs::msg::RobotIntent> edge_fleet_;
  std::map<std::string, edge_fleet_msgs::msg::RobotIntent> trad_fleet_;

  std::map<std::string, uint32_t> edge_laps_;
  std::map<std::string, uint32_t> trad_laps_;

  bool edge_finished_{false};
  bool trad_finished_{false};
  float edge_finish_time_sec_{0.0f};
  float trad_finish_time_sec_{0.0f};
  float trad_cumulative_idle_sec_{0.0f};
  uint32_t edge_collision_count_{0};
  uint32_t trad_collision_count_{0};

  // ROS 2 Interfaces
  rclcpp::Subscription<edge_fleet_msgs::msg::RobotIntent>::SharedPtr intent_sub_;
  rclcpp::Subscription<edge_fleet_msgs::msg::ConflictEvent>::SharedPtr conflict_sub_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr> lap_subs_;

  rclcpp::Publisher<edge_fleet_msgs::msg::RaceTelemetry>::SharedPtr race_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace edge_fleet_core

#endif  // EDGE_FLEET_CORE_RACE_MANAGER_NODE_HPP_
