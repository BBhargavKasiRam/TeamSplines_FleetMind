#!/usr/bin/env python3
"""
###############################################################################
#                           MOCK CONTROLLER                                   #
#                    FOR SIMULATION TESTING ONLY                              #
#                                                                             #
# This mock controller is provided STRICTLY to demonstrate simulation         #
# functionality, verify robot motion in Gazebo, and validate task handling.   #
#                                                                             #
# When connecting your friend's logic:                                        #
# 1. Disable this mock controller: pass use_mock_controller:=false             #
# 2. Or connect friend's logic directly to /logic/<ns>/waypoint or cmd_vel    #
###############################################################################
"""

import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Path
from warehouse_3d_interfaces.msg import WarehouseTask, RobotState, RobotTaskAssignment
from warehouse_3d_interfaces.srv import AssignTask

class MockController(Node):
    def __init__(self):
        super().__init__('mock_controller')
        self.get_logger().info("=" * 60)
        self.get_logger().info(" MOCK CONTROLLER - FOR SIMULATION TESTING ONLY")
        self.get_logger().info(" Demonstrating Gazebo motion and adapter interfaces")
        self.get_logger().info("=" * 60)

        self.robot_namespaces = [
            "robot_alpha",
            "robot_beta",
            "robot_gamma",
            "robot_delta",
            "robot_epsilon"
        ]

        # Waypoint publishers to logic adapter
        self.waypoint_pubs = {}
        self.path_pubs = {}
        for ns in self.robot_namespaces:
            self.waypoint_pubs[ns] = self.create_publisher(PoseStamped, f'/logic/{ns}/waypoint', 10)
            self.path_pubs[ns] = self.create_publisher(Path, f'/logic/{ns}/path', 10)

        # Assignment publisher
        self.assign_pub = self.create_publisher(RobotTaskAssignment, '/logic/assign_task', 10)

        # State tracking
        self.robot_states = {}
        for ns in self.robot_namespaces:
            self.create_subscription(
                RobotState,
                f'/{ns}/robot_state',
                lambda msg, r_ns=ns: self.robot_state_callback(r_ns, msg),
                10
            )

        self.pending_tasks = []
        self.create_subscription(
            WarehouseTask,
            '/warehouse/task_stream',
            self.task_callback,
            20
        )

        # Base home coordinates for each robot
        self.robot_base_coords = {
            "robot_alpha": (-12.5, -2.0),
            "robot_beta": (-12.5, -1.0),
            "robot_gamma": (-12.5, 0.0),
            "robot_delta": (-12.5, 1.0),
            "robot_epsilon": (-12.5, 2.0)
        }

        # Fast dispatch initialization so the user sees the robots moving immediately
        self.start_delay = 0.5 # seconds
        self.start_time = None

        # Robot assignment status
        self.robot_active_task = {ns: None for ns in self.robot_namespaces}
        self.robot_stage = {ns: "IDLE" for ns in self.robot_namespaces} # IDLE, TO_PICKUP, TO_DEST, TO_BASE

        # Periodic test dispatch timer (every 0.5s)
        self.timer = self.create_timer(0.5, self.test_dispatch_loop)

    def robot_state_callback(self, ns, msg: RobotState):
        self.robot_states[ns] = msg

    def task_callback(self, msg: WarehouseTask):
        if msg.status == WarehouseTask.STATUS_PENDING:
            if not any(t.task_id == msg.task_id for t in self.pending_tasks):
                self.pending_tasks.append(msg)

    def generate_safe_path(self, start_x, start_y, goal_x, goal_y):
        """Generates waypoint sequence routing cleanly along the West warehouse transit corridor."""
        pts = []
        # If start is in the docking bay, pull out into the transit corridor first
        if start_x < -11.5:
            pts.append((-11.0, start_y))
        # Move along transit corridor to goal's Y coordinate
        pts.append((-11.0, goal_y))
        # Drive down the aisle/station to goal
        pts.append((goal_x, goal_y))
        return pts

    def test_dispatch_loop(self):
        """Mock loop that assigns pending tasks to idle robots with safe aisle routing."""
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if self.start_time is None:
            self.start_time = now_sec
            return
        if (now_sec - self.start_time) < self.start_delay:
            return

        for ns in self.robot_namespaces:
            if self.robot_active_task[ns] is None and self.pending_tasks:
                task = self.pending_tasks.pop(0)
                self.robot_active_task[ns] = task
                self.robot_stage[ns] = "TO_PICKUP"

                # Publish assignment to logic adapter
                assign_msg = RobotTaskAssignment()
                assign_msg.robot_id = ns
                assign_msg.task_id = task.task_id
                assign_msg.action_type = RobotTaskAssignment.ACTION_ASSIGN
                self.assign_pub.publish(assign_msg)

                self.get_logger().info(
                    f"[MOCK] Assigned Task {task.task_id} (Priority: {task.priority}) to {ns}"
                )

                # Get robot current coordinates
                r_state = self.robot_states.get(ns)
                rx = r_state.current_pose.position.x if r_state else -12.5
                ry = r_state.current_pose.position.y if r_state else 0.0

                # Send initial safe aisle path to pickup location
                target_pose = task.pickup_pose
                path_pts = self.generate_safe_path(rx, ry, target_pose.position.x, target_pose.position.y)
                self.send_path(ns, path_pts)
                # Dispatch one robot per cycle for clean traffic spacing
                break

            elif self.robot_active_task[ns] is not None:
                # Check if reached target
                r_state = self.robot_states.get(ns)
                if r_state:
                    rx = r_state.current_pose.position.x
                    ry = r_state.current_pose.position.y
                    task = self.robot_active_task[ns]

                    if self.robot_stage[ns] == "TO_PICKUP":
                        px = task.pickup_pose.position.x
                        py = task.pickup_pose.position.y
                        dist = math.hypot(px - rx, py - ry)
                        if dist < 0.65:
                            self.get_logger().info(f"[MOCK] {ns} reached pickup {task.pickup_location}. Routing to destination {task.destination_location}")
                            self.robot_stage[ns] = "TO_DEST"
                            dest_pts = self.generate_safe_path(rx, ry, task.destination_pose.position.x, task.destination_pose.position.y)
                            self.send_path(ns, dest_pts)

                    elif self.robot_stage[ns] == "TO_DEST":
                        dx = task.destination_pose.position.x
                        dy = task.destination_pose.position.y
                        dist = math.hypot(dx - rx, dy - ry)
                        if dist < 0.65:
                            self.get_logger().info(f"[MOCK] {ns} completed delivery to {task.destination_location} for {task.task_id}! Routing back to base...")
                            self.robot_stage[ns] = "TO_BASE"
                            base_x, base_y = self.robot_base_coords.get(ns, (-12.5, 0.0))
                            base_pts = self.generate_safe_path(rx, ry, base_x, base_y)
                            self.send_path(ns, base_pts)

                    elif self.robot_stage[ns] == "TO_BASE":
                        base_x, base_y = self.robot_base_coords.get(ns, (-12.5, 0.0))
                        dist = math.hypot(base_x - rx, base_y - ry)
                        if dist < 0.65:
                            self.get_logger().info(f"[MOCK] {ns} successfully returned and docked at base ({base_x}, {base_y}). Awaiting next task.")
                            self.robot_active_task[ns] = None
                            self.robot_stage[ns] = "IDLE"

    def send_path(self, ns, points):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        for pt in points:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(pt[0])
            ps.pose.position.y = float(pt[1])
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pubs[ns].publish(path)

    def send_waypoint(self, ns, x, y):
        wp = PoseStamped()
        wp.header.stamp = self.get_clock().now().to_msg()
        wp.header.frame_id = 'map'
        wp.pose.position.x = float(x)
        wp.pose.position.y = float(y)
        wp.pose.position.z = 0.0
        wp.pose.orientation.w = 1.0
        self.waypoint_pubs[ns].publish(wp)

def main(args=None):
    rclpy.init(args=args)
    node = MockController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
