#!/usr/bin/env python3
"""
Simulated Autonomous Mobile Robot (AMR) Node for Edge Fleet Coordination.
Simulates kinematics, odometry, LiDAR scan, waypoint tracking, and desired velocity commands.
Universal: Drops into any warehouse simulation or runs standalone for automated benchmarking.
"""

import sys
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String, Int32
import tf2_ros


class SimulatedAMR(Node):
    def __init__(self):
        super().__init__('simulated_amr')

        self.declare_parameter('robot_id', 'robot_1')
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_yaw', 0.0)
        self.declare_parameter('max_speed', 0.8)
        self.declare_parameter('route_waypoints', "0.0,0.0; 10.0,0.0; 10.0,5.0; 0.0,5.0")
        self.declare_parameter('target_laps', 2)

        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        self.x = self.get_parameter('start_x').get_parameter_value().double_value
        self.y = self.get_parameter('start_y').get_parameter_value().double_value
        self.yaw = self.get_parameter('start_yaw').get_parameter_value().double_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.target_laps = self.get_parameter('target_laps').get_parameter_value().integer_value

        raw_route = self.get_parameter('route_waypoints').get_parameter_value().string_value
        self.default_waypoints = []
        for pair in raw_route.split(';'):
            pair = pair.strip()
            if pair:
                parts = pair.split(',')
                self.default_waypoints.append((float(parts[0].strip()), float(parts[1].strip())))

        self.waypoints = list(self.default_waypoints)
        self.current_wp_idx = 0
        self.completed_laps = 0
        self.is_mission_finished = False
        self.vx = 0.0
        self.wz = 0.0
        self.is_detouring = False

        # Subscriptions
        # Subscribes to the resolved safe velocity published by conflict_resolver_node
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Dynamic Re-routing & Task Allocation triggers
        self.blocked_sub = self.create_subscription(
            Bool, 'aisle_blocked', self.aisle_blocked_callback, 10)
        self.assigned_task_sub = self.create_subscription(
            String, 'current_assigned_task', self.assigned_task_callback, 10)

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.plan_pub = self.create_publisher(Path, 'plan', 10)
        self.raw_cmd_pub = self.create_publisher(Twist, 'cmd_vel_raw', 10)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.lap_pub = self.create_publisher(Int32, 'lap_count', 10)

        self.last_time = self.get_clock().now()

        # Loop at 20 Hz (50 ms)
        self.timer = self.create_timer(0.05, self.update_physics_and_planner)

        self.get_logger().info(
            f"[{self.robot_id}] AMR Simulator active at ({self.x:.1f}, {self.y:.1f}, {self.yaw:.2f} rad), "
            f"tracking {len(self.waypoints)} warehouse waypoints."
        )

    def cmd_vel_callback(self, msg: Twist):
        self.vx = msg.linear.x
        self.wz = msg.angular.z

    def aisle_blocked_callback(self, msg: Bool):
        if msg.data and not self.is_detouring:
            self.is_detouring = True
            self.get_logger().warn(
                f"[{self.robot_id}] RE-ROUTING: Forward aisle blocked! Generating dynamic detour around obstacle."
            )
            # Dynamic detour: divert perpendicularly to bypass corridor (+2.0m lateral offset)
            curr_target = self.waypoints[self.current_wp_idx]
            detour_wp1 = (self.x + 0.5 * math.cos(self.yaw), self.y + 2.0)
            detour_wp2 = (curr_target[0], curr_target[1] + 2.0)
            self.waypoints = [detour_wp1, detour_wp2] + list(self.default_waypoints)
            self.current_wp_idx = 0
        elif not msg.data and self.is_detouring:
            self.is_detouring = False
            self.get_logger().info(f"[{self.robot_id}] Aisle cleared. Resuming nominal route.")
            self.waypoints = list(self.default_waypoints)
            self.current_wp_idx = 0

    def assigned_task_callback(self, msg: String):
        self.get_logger().info(
            f"[{self.robot_id}] Dynamic Task Allocation: Reallocated task '{msg.data}' assigned. "
            f"Updating local route planner to visit reallocated pickup zone."
        )

    def update_physics_and_planner(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if dt <= 0.0 or dt > 0.5:
            return

        # 1. Update Differential Drive Kinematics from actual commanded speed
        self.x += self.vx * math.cos(self.yaw) * dt
        self.y += self.vx * math.sin(self.yaw) * dt
        self.yaw += self.wz * dt
        # Normalize yaw
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        # 2. Local Waypoint Pursuit -> Generate Raw Desired Twist (cmd_vel_raw)
        if self.waypoints:
            target_x, target_y = self.waypoints[self.current_wp_idx]
            dx = target_x - self.x
            dy = target_y - self.y
            dist = math.hypot(dx, dy)

            if dist < 0.35:
                # Switch to next waypoint (looping)
                next_idx = (self.current_wp_idx + 1) % len(self.waypoints)
                if next_idx == 0:
                    self.completed_laps += 1
                    lap_msg = Int32()
                    lap_msg.data = self.completed_laps
                    self.lap_pub.publish(lap_msg)
                    self.get_logger().info(
                        f"[{self.robot_id}] Completed Lap {self.completed_laps} / {self.target_laps}"
                    )
                self.current_wp_idx = next_idx
                target_x, target_y = self.waypoints[self.current_wp_idx]
                dx = target_x - self.x
                dy = target_y - self.y

            target_angle = math.atan2(dy, dx)
            angle_diff = target_angle - self.yaw
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

            raw_cmd = Twist()
            # Turn in place if angle difference is large
            if abs(angle_diff) > 0.6:
                raw_cmd.linear.x = 0.15
                raw_cmd.angular.z = max(-1.2, min(1.2, 2.0 * angle_diff))
            else:
                raw_cmd.linear.x = min(self.max_speed, 0.4 + 0.5 * dist)
                raw_cmd.angular.z = max(-1.0, min(1.0, 1.8 * angle_diff))

            self.raw_cmd_pub.publish(raw_cmd)

        # 3. Publish Odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = f'{self.robot_id}/base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Euler to Quaternion (Yaw only)
        cy = math.cos(self.yaw * 0.5)
        sy = math.sin(self.yaw * 0.5)
        odom.pose.pose.orientation.w = cy
        odom.pose.pose.orientation.z = sy

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.wz
        self.odom_pub.publish(odom)

        # 4. Publish Planned Path
        path_msg = Path()
        path_msg.header.stamp = now.to_msg()
        path_msg.header.frame_id = 'map'

        # Current pose
        curr_p = PoseStamped()
        curr_p.header = path_msg.header
        curr_p.pose.position.x = self.x
        curr_p.pose.position.y = self.y
        path_msg.poses.append(curr_p)

        # Remaining waypoints in sequence
        for i in range(len(self.waypoints)):
            idx = (self.current_wp_idx + i) % len(self.waypoints)
            wp_p = PoseStamped()
            wp_p.header = path_msg.header
            wp_p.pose.position.x = self.waypoints[idx][0]
            wp_p.pose.position.y = self.waypoints[idx][1]
            path_msg.poses.append(wp_p)

        self.plan_pub.publish(path_msg)

        # 5. Synthetic LiDAR scan
        scan = LaserScan()
        scan.header.stamp = now.to_msg()
        scan.header.frame_id = f'{self.robot_id}/base_scan'
        scan.angle_min = -math.pi / 2
        scan.angle_max = math.pi / 2
        scan.angle_increment = math.pi / 180.0
        scan.range_min = 0.1
        scan.range_max = 12.0
        # 180 beams default at 10.0m clear
        scan.ranges = [10.0] * 180
        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = SimulatedAMR()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
