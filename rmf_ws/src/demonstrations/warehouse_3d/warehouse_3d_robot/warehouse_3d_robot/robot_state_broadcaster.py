#!/usr/bin/env python3
"""
Robot State Broadcaster Node
Aggregates robot states, handles global TF transformations, and publishes RobotState messages.
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Pose, Twist
from nav_msgs.msg import Odometry
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from warehouse_3d_interfaces.msg import RobotState
from warehouse_3d_interfaces.srv import SetRobotState

ROBOT_FLEET = [
    {"name": "Alpha", "color": "Blue", "ns": "robot_alpha", "id": 1, "x": -12.5, "y": -2.0, "z": 0.0, "yaw": 0.0},
    {"name": "Beta", "color": "Red", "ns": "robot_beta", "id": 2, "x": -12.5, "y": -1.0, "z": 0.0, "yaw": 0.0},
    {"name": "Gamma", "color": "Green", "ns": "robot_gamma", "id": 3, "x": -12.5, "y": 0.0, "z": 0.0, "yaw": 0.0},
    {"name": "Delta", "color": "Yellow", "ns": "robot_delta", "id": 4, "x": -12.5, "y": 1.0, "z": 0.0, "yaw": 0.0},
    {"name": "Epsilon", "color": "Purple", "ns": "robot_epsilon", "id": 5, "x": -12.5, "y": 2.0, "z": 0.0, "yaw": 0.0},
]

class RobotTracker:
    def __init__(self, node, info):
        self.node = node
        self.info = info
        self.ns = info['ns']
        self.name = info['name']
        self.color = info['color']
        self.id = str(info['id'])

        # State fields
        self.status = "IDLE"
        self.current_pose = Pose()
        self.current_pose.position.x = info['x']
        self.current_pose.position.y = info['y']
        self.current_pose.position.z = 0.1
        self.current_pose.orientation.w = 1.0
        self.current_velocity = Twist()
        self.battery = 98.5
        self.current_task_id = ""
        self.carried_package_id = ""
        self.is_carrying = False

        # Individual state publisher
        self.state_pub = node.create_publisher(RobotState, f'/{self.ns}/robot_state', 10)

        # Odometry subscriber
        self.odom_sub = node.create_subscription(
            Odometry,
            f'/{self.ns}/odom',
            self.odom_callback,
            10
        )

        # Dynamic TF Broadcaster for base_link to sensors if needed
        self.tf_broadcaster = TransformBroadcaster(node)

    def odom_callback(self, msg: Odometry):
        # Local odom in robot frame -> compute global pose relative to map
        # Given static anchor at (info['x'], info['y'])
        lx = msg.pose.pose.position.x
        ly = msg.pose.pose.position.y
        lz = msg.pose.pose.position.z

        # In map frame:
        self.current_pose.position.x = self.info['x'] + lx
        self.current_pose.position.y = self.info['y'] + ly
        self.current_pose.position.z = lz + 0.1
        self.current_pose.orientation = msg.pose.pose.orientation

        self.current_velocity = msg.twist.twist
        speed = math.sqrt(self.current_velocity.linear.x**2 + self.current_velocity.linear.y**2)

        if speed > 0.05:
            if self.status != "DEADLOCKED" and self.status != "YIELDING":
                self.status = "MOVING"
        elif self.status == "MOVING":
            self.status = "IDLE"

        # Drain battery slightly when moving
        if speed > 0.05 and self.battery > 5.0:
            self.battery -= 0.001

        # Also broadcast dynamic TF from odom to base_footprint if simulator doesn't
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = f"{self.ns}/odom"
        t.child_frame_id = f"{self.ns}/base_footprint"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

        self.publish_state()

    def publish_state(self):
        msg = RobotState()
        msg.robot_id = self.id
        msg.robot_name = self.name
        msg.robot_namespace = self.ns
        msg.status = self.status
        msg.current_pose = self.current_pose
        msg.current_velocity = self.current_velocity
        msg.battery_percentage = float(self.battery)
        msg.current_task_id = self.current_task_id
        msg.carried_package_id = self.carried_package_id
        msg.is_carrying_package = self.is_carrying
        msg.assigned_color = self.color
        self.state_pub.publish(msg)
        return msg

class RobotStateBroadcaster(Node):
    def __init__(self):
        super().__init__('robot_state_broadcaster')
        self.declare_parameter('robot_count', 5)
        self.robot_count = self.get_parameter('robot_count').get_parameter_value().integer_value

        self.get_logger().info(f"Starting Robot State Broadcaster for {self.robot_count} AMRs...")

        # Static TF broadcaster (publishes map -> <ns>/odom static anchors)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # Trackers
        self.trackers = []
        active_fleet = ROBOT_FLEET[:max(1, min(self.robot_count, len(ROBOT_FLEET)))]
        for rob in active_fleet:
            tracker = RobotTracker(self, rob)
            self.trackers.append(tracker)

        self.publish_static_map_transforms(active_fleet)

        # Periodic timer for status broadcast
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Service to update robot state (yield, resume, stop)
        self.srv = self.create_service(
            SetRobotState,
            '/warehouse/set_robot_state',
            self.handle_set_robot_state
        )

    def publish_static_map_transforms(self, active_fleet):
        static_transforms = []
        now = self.get_clock().now().to_msg()
        for rob in active_fleet:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'map'
            t.child_frame_id = f"{rob['ns']}/odom"
            t.transform.translation.x = rob['x']
            t.transform.translation.y = rob['y']
            t.transform.translation.z = 0.0
            t.transform.rotation.w = 1.0
            static_transforms.append(t)
        self.static_tf_broadcaster.sendTransform(static_transforms)
        self.get_logger().info(f"Published static TF anchors from 'map' to {len(static_transforms)} robot odom frames.")

    def timer_callback(self):
        for tracker in self.trackers:
            tracker.publish_state()

    def handle_set_robot_state(self, request, response):
        target_id = request.robot_id
        cmd = request.command
        found = False
        for t in self.trackers:
            if t.id == target_id or t.name.lower() == target_id.lower() or t.ns == target_id:
                found = True
                if cmd in ["IDLE", "MOVING", "PICKING", "DROPPING", "YIELDING", "WAITING", "CHARGING", "DEADLOCKED"]:
                    t.status = cmd
                    response.success = True
                    response.message = f"Robot {t.name} state set to {cmd}"
                else:
                    response.success = False
                    response.message = f"Unknown command: {cmd}"
                break
        if not found:
            response.success = False
            response.message = f"Robot {target_id} not found"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = RobotStateBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
