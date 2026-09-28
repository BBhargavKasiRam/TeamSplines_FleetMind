"""
Warehouse Logic Adapter - Python API Client
Lightweight integration library enabling external logic scripts to easily interact
with the 3D warehouse simulation without manual ROS 2 subscriber/publisher wiring.
"""

import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
from warehouse_3d_interfaces.msg import RobotState, WarehouseTask, RobotTaskAssignment, DeadlockStatus

class WarehouseLogicClient(Node):
    """
    Python client for external autonomous algorithms.
    Provides direct access to robot positions, task queues, and movement commands.
    """
    def __init__(self, node_name="warehouse_logic_client"):
        if not rclpy.ok():
            rclpy.init()
        super().__init__(node_name)

        self.robot_namespaces = [
            "robot_alpha", "robot_beta", "robot_gamma", "robot_delta", "robot_epsilon"
        ]

        self.robot_states = {}
        self.active_tasks = {}
        self.deadlock_info = None

        # Subscribers
        for ns in self.robot_namespaces:
            self.create_subscription(
                RobotState,
                f'/{ns}/robot_state',
                lambda msg, r=ns: self._on_robot_state(r, msg),
                10
            )

        self.create_subscription(
            WarehouseTask,
            '/warehouse/task_stream',
            self._on_task,
            20
        )

        self.create_subscription(
            DeadlockStatus,
            '/warehouse/deadlock_status',
            self._on_deadlock,
            10
        )

        # Command publishers
        self.vel_pubs = {}
        self.waypoint_pubs = {}
        self.control_pubs = {}
        for ns in self.robot_namespaces:
            self.vel_pubs[ns] = self.create_publisher(Twist, f'/logic/{ns}/cmd_vel', 10)
            self.waypoint_pubs[ns] = self.create_publisher(PoseStamped, f'/logic/{ns}/waypoint', 10)
            self.control_pubs[ns] = self.create_publisher(String, f'/logic/{ns}/control', 10)

        self.assign_pub = self.create_publisher(RobotTaskAssignment, '/logic/assign_task', 10)

        # Background thread to spin
        self._thread = threading.Thread(target=self._spin_worker, daemon=True)
        self._thread.start()

    def _spin_worker(self):
        rclpy.spin(self)

    def _on_robot_state(self, ns, msg):
        self.robot_states[ns] = msg

    def _on_task(self, msg):
        self.active_tasks[msg.task_id] = msg

    def _on_deadlock(self, msg):
        self.deadlock_info = msg

    # --- External Logic Helper Methods ---

    def get_robot_state(self, ns: str):
        return self.robot_states.get(ns)

    def get_all_robot_states(self):
        return dict(self.robot_states)

    def get_tasks(self):
        return dict(self.active_tasks)

    def get_deadlock_status(self):
        return self.deadlock_info

    def send_velocity(self, ns: str, linear_x: float, angular_z: float):
        """Option A: Direct velocity control"""
        if ns in self.vel_pubs:
            t = Twist()
            t.linear.x = float(linear_x)
            t.angular.z = float(angular_z)
            self.vel_pubs[ns].publish(t)

    def send_waypoint(self, ns: str, x: float, y: float):
        """Option B: Waypoint coordinate"""
        if ns in self.waypoint_pubs:
            p = PoseStamped()
            p.header.stamp = self.get_clock().now().to_msg()
            p.header.frame_id = 'map'
            p.pose.position.x = float(x)
            p.pose.position.y = float(y)
            p.pose.orientation.w = 1.0
            self.waypoint_pubs[ns].publish(p)

    def assign_task(self, ns: str, task_id: str):
        """Option D: Task assignment"""
        msg = RobotTaskAssignment()
        msg.robot_id = ns
        msg.task_id = task_id
        msg.action_type = RobotTaskAssignment.ACTION_ASSIGN
        self.assign_pub.publish(msg)

    def yield_robot(self, ns: str):
        """Command robot to yield in conflict / deadlock situation"""
        if ns in self.control_pubs:
            msg = String()
            msg.data = "YIELD"
            self.control_pubs[ns].publish(msg)

    def resume_robot(self, ns: str):
        """Resume robot from yield/pause"""
        if ns in self.control_pubs:
            msg = String()
            msg.data = "RESUME"
            self.control_pubs[ns].publish(msg)
