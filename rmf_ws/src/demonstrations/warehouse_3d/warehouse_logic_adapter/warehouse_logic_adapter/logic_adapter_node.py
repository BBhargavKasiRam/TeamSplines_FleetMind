#!/usr/bin/env python3
"""
Warehouse Logic Adapter Node
The primary bridge between external autonomous robot logic and ROS 2 / Gazebo Sim.

Supports:
- Option A: Direct velocity commands (geometry_msgs/Twist)
- Option B: Waypoint steering (PoseStamped / PoseArray)
- Option C: Nav path tracking (nav_msgs/Path)
- Option D: Task assignment (RobotTaskAssignment / AssignTask service)
- Robot coordination signals: YIELD, WAIT, RESUME, STOP, REROUTE
- Deadlock / proximity exposure to external logic
"""

import math
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseArray, Pose
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import String
from warehouse_3d_interfaces.msg import (
    RobotState, WarehouseTask, RobotTaskAssignment, DeadlockStatus
)
from warehouse_3d_interfaces.srv import AssignTask, SetRobotState

ROBOT_NAMESPACES = [
    "robot_alpha",
    "robot_beta",
    "robot_gamma",
    "robot_delta",
    "robot_epsilon"
]

class RobotControllerBridge:
    """Manages command dispatch and waypoint/path tracking for a single AMR."""
    def __init__(self, node, ns):
        self.node = node
        self.ns = ns

        # Commanded movement targets
        self.active_waypoint = None
        self.active_path = []
        self.yield_state = False
        self.is_paused = False

        # Current estimated global pose and velocity
        self.global_pose = Pose()
        self.current_twist = Twist()
        self.robot_state = None

        # Robot velocity publisher to Gazebo
        self.cmd_vel_pub = node.create_publisher(Twist, f'/{ns}/cmd_vel', 10)

        # State subscriber
        self.state_sub = node.create_subscription(
            RobotState,
            f'/{ns}/robot_state',
            self.state_callback,
            10
        )

        # External logic command inputs:
        # Option A: direct velocity
        self.logic_vel_sub = node.create_subscription(
            Twist,
            f'/logic/{ns}/cmd_vel',
            self.logic_vel_callback,
            10
        )

        # Option B: waypoints
        self.waypoint_sub = node.create_subscription(
            PoseStamped,
            f'/logic/{ns}/waypoint',
            self.waypoint_callback,
            10
        )

        # Option C: nav paths
        self.path_sub = node.create_subscription(
            Path,
            f'/logic/{ns}/path',
            self.path_callback,
            10
        )

        # Coordination commands (WAIT, RESUME, YIELD)
        self.control_sub = node.create_subscription(
            String,
            f'/logic/{ns}/control',
            self.control_callback,
            10
        )

    def state_callback(self, msg: RobotState):
        self.robot_state = msg
        self.global_pose = msg.current_pose
        self.current_twist = msg.current_velocity

    def logic_vel_callback(self, msg: Twist):
        # Option A: direct velocity feedthrough
        if not self.yield_state and not self.is_paused:
            self.cmd_vel_pub.publish(msg)

    def waypoint_callback(self, msg: PoseStamped):
        # Option B: Set single waypoint (only override if no active path)
        if not self.active_path:
            self.active_waypoint = msg.pose

    def path_callback(self, msg: Path):
        # Option C: Set series of path waypoints
        self.active_path = [p.pose for p in msg.poses]
        if self.active_path:
            self.active_waypoint = self.active_path.pop(0)

    def control_callback(self, msg: String):
        cmd = msg.data.upper()
        if cmd == "YIELD":
            self.yield_state = True
            self.stop_robot()
            self.node.get_logger().info(f"[{self.ns}] External logic commanded YIELD")
        elif cmd == "WAIT" or cmd == "PAUSE":
            self.is_paused = True
            self.stop_robot()
            self.node.get_logger().info(f"[{self.ns}] External logic commanded WAIT")
        elif cmd == "RESUME":
            self.yield_state = False
            self.is_paused = False
            self.node.get_logger().info(f"[{self.ns}] External logic commanded RESUME")
        elif cmd == "STOP":
            self.active_waypoint = None
            self.active_path = []
            self.stop_robot()

    def stop_robot(self):
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

    def update_waypoint_tracking(self):
        """Pure pursuit / proportional tracking to guide robot along logic-assigned waypoints."""
        if self.yield_state or self.is_paused or self.active_waypoint is None:
            return

        rx = self.global_pose.position.x
        ry = self.global_pose.position.y
        gx = self.active_waypoint.position.x
        gy = self.active_waypoint.position.y

        dx = gx - rx
        dy = gy - ry
        dist = math.hypot(dx, dy)

        # Heading angle
        target_heading = math.atan2(dy, dx)

        # Extract current yaw from quaternion
        q = self.global_pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        curr_yaw = math.atan2(siny_cosp, cosy_cosp)

        heading_error = target_heading - curr_yaw
        # Normalize to [-pi, pi]
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi

        twist = Twist()
        if dist < 0.45:
            # Reached waypoint - advance to next waypoint smoothly
            if self.active_path:
                self.active_waypoint = self.active_path.pop(0)
                dx = self.active_waypoint.position.x - rx
                dy = self.active_waypoint.position.y - ry
                dist = math.hypot(dx, dy)
                target_heading = math.atan2(dy, dx)
                heading_error = target_heading - curr_yaw
                while heading_error > math.pi: heading_error -= 2 * math.pi
                while heading_error < -math.pi: heading_error += 2 * math.pi
            else:
                self.active_waypoint = None
                self.stop_robot()
                return

        # Continuous smooth navigation with active responsive pace
        if abs(heading_error) > 0.45:
            twist.angular.z = max(-1.8, min(1.8, 2.2 * heading_error))
            twist.linear.x = 0.15 # slight forward creep for smooth natural turning
        else:
            twist.angular.z = max(-1.2, min(1.2, 1.6 * heading_error))
            twist.linear.x = max(0.8, min(2.0, 1.8 * dist))

        self.cmd_vel_pub.publish(twist)

class WarehouseLogicAdapterNode(Node):
    def __init__(self):
        super().__init__('warehouse_logic_adapter')
        self.get_logger().info("Initializing Warehouse Logic Adapter (External Integration Gateway)...")

        # Create bridge instances for each robot
        self.robots = {}
        for ns in ROBOT_NAMESPACES:
            self.robots[ns] = RobotControllerBridge(self, ns)

        # Tasks registry
        self.tasks = {}
        self.task_sub = self.create_subscription(
            WarehouseTask,
            '/warehouse/task_stream',
            self.task_callback,
            20
        )

        # External Logic: Option D (Task Assignment)
        self.task_assign_sub = self.create_subscription(
            RobotTaskAssignment,
            '/logic/assign_task',
            self.logic_assign_task_callback,
            10
        )

        # Service client to task generator
        self.assign_client = self.create_client(AssignTask, '/warehouse/assign_task')

        # Deadlock and Fleet State Publishers (exposing full system state to friend's logic)
        self.fleet_state_pub = self.create_publisher(String, '/warehouse/fleet_state', 10)
        self.deadlock_pub = self.create_publisher(DeadlockStatus, '/warehouse/deadlock_status', 10)

        # Periodic loop: waypoint steering & state aggregation (20 Hz)
        self.loop_timer = self.create_timer(0.05, self.update_loop)

    def task_callback(self, msg: WarehouseTask):
        self.tasks[msg.task_id] = msg

    def logic_assign_task_callback(self, msg: RobotTaskAssignment):
        # Option D: External logic assigns a task to a robot
        self.get_logger().info(f"Logic commanded assignment: Robot {msg.robot_id} -> Task {msg.task_id}")
        if self.assign_client.service_is_ready():
            req = AssignTask.Request()
            req.robot_id = msg.robot_id
            req.task_id = msg.task_id
            self.assign_client.call_async(req)

    def update_loop(self):
        # Update waypoint tracking for all robots
        for bridge in self.robots.values():
            bridge.update_waypoint_tracking()

        # Check for proximity conflicts and publish deadlock exposure status
        self.evaluate_conflicts_for_logic()

        # Periodically publish fleet JSON state snapshot (5 Hz)
        if hasattr(self, 'tick_count'):
            self.tick_count += 1
        else:
            self.tick_count = 0

        if self.tick_count % 4 == 0:
            self.publish_fleet_snapshot()

    def evaluate_conflicts_for_logic(self):
        """Exposes proximity / deadlock conditions to friend's logic without resolving them."""
        robot_list = list(self.robots.values())
        involved = []
        conflicts = []
        proximity_threshold = 1.6 # meters

        for i in range(len(robot_list)):
            for j in range(i + 1, len(robot_list)):
                r1 = robot_list[i]
                r2 = robot_list[j]
                p1 = r1.global_pose.position
                p2 = r2.global_pose.position
                dist = math.hypot(p1.x - p2.x, p1.y - p2.y)

                if dist < proximity_threshold:
                    involved.extend([r1.ns, r2.ns])
                    loc_desc = f"Between {r1.ns} ({p1.x:.1f},{p1.y:.1f}) and {r2.ns} ({p2.x:.1f},{p2.y:.1f}) dist={dist:.2f}m"
                    conflicts.append(loc_desc)

        dl_msg = DeadlockStatus()
        dl_msg.timestamp = self.get_clock().now().to_msg()
        if involved:
            dl_msg.deadlock_detected = True
            dl_msg.involved_robots = list(set(involved))
            dl_msg.conflict_locations = conflicts
            dl_msg.suggested_resolution = "EXPOSED_TO_EXTERNAL_LOGIC: Awaiting yield/reroute command"
        else:
            dl_msg.deadlock_detected = False
            dl_msg.involved_robots = []
            dl_msg.conflict_locations = []
            dl_msg.suggested_resolution = "NOMINAL_OPERATION"

        self.deadlock_pub.publish(dl_msg)

    def publish_fleet_snapshot(self):
        snapshot = {
            "timestamp": self.get_clock().now().nanoseconds / 1e9,
            "robots": {},
            "tasks": {}
        }
        for ns, b in self.robots.items():
            snapshot["robots"][ns] = {
                "x": round(b.global_pose.position.x, 3),
                "y": round(b.global_pose.position.y, 3),
                "status": b.robot_state.status if b.robot_state else "UNKNOWN",
                "battery": round(b.robot_state.battery_percentage, 1) if b.robot_state else 100.0,
                "current_task": b.robot_state.current_task_id if b.robot_state else "",
                "yield_state": b.yield_state
            }
        for tid, t in self.tasks.items():
            snapshot["tasks"][tid] = {
                "priority": t.priority,
                "pkg": t.package_id,
                "from": t.pickup_location,
                "to": t.destination_location,
                "status": t.status,
                "assigned_robot": t.assigned_robot
            }
        s_msg = String()
        s_msg.data = json.dumps(snapshot)
        self.fleet_state_pub.publish(s_msg)

def main(args=None):
    rclpy.init(args=args)
    node = WarehouseLogicAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
