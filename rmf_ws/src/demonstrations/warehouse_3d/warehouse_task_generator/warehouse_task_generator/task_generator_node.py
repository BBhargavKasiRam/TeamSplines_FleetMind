#!/usr/bin/env python3
"""
Dynamic Task Generator Node
Generates and manages multi-task queues with dynamic priorities, pickup locations, and destinations.
Exposes full task state to the warehouse logic adapter for external priority/assignment algorithms.
"""

import random
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from warehouse_3d_interfaces.msg import WarehouseTask, TaskState
from warehouse_3d_interfaces.srv import AssignTask
from warehouse_task_generator.warehouse_locations import (
    WAREHOUSE_LOCATIONS, PICKUP_LOCATIONS, DESTINATION_LOCATIONS, get_location_pose
)

PRIORITY_LEVELS = ["LOW", "MEDIUM", "HIGH", "CRITICAL"]

class TaskGeneratorNode(Node):
    def __init__(self):
        super().__init__('warehouse_task_generator')

        # Declare configurable parameters
        self.declare_parameter('robot_count', 5)
        self.declare_parameter('package_count', 30)
        self.declare_parameter('task_count', 10)
        self.declare_parameter('continuous_generation', False)
        self.declare_parameter('generation_interval', 15.0)

        self.robot_count = self.get_parameter('robot_count').get_parameter_value().integer_value
        self.package_count = self.get_parameter('package_count').get_parameter_value().integer_value
        self.task_count = self.get_parameter('task_count').get_parameter_value().integer_value
        self.continuous = self.get_parameter('continuous_generation').get_parameter_value().bool_value
        self.interval = self.get_parameter('generation_interval').get_parameter_value().double_value

        self.get_logger().info(
            f"Initializing Task Generator: robots={self.robot_count}, "
            f"packages={self.package_count}, initial_tasks={self.task_count}"
        )

        # Publishers
        self.tasks_pub = self.create_publisher(WarehouseTask, '/warehouse/task_stream', 20)
        self.task_summary_pub = self.create_publisher(String, '/warehouse/task_summary', 10)

        # Services
        self.assign_srv = self.create_service(
            AssignTask,
            '/warehouse/assign_task',
            self.handle_assign_task
        )

        # Internal task registry
        self.tasks = {}
        self.task_counter = 1

        # Seed initial tasks
        self.generate_initial_tasks(self.task_count)

        # Periodic publication of active tasks
        self.publish_timer = self.create_timer(1.0, self.publish_all_tasks)

        # Continuous generation timer (if enabled)
        if self.continuous:
            self.gen_timer = self.create_timer(self.interval, self.generate_single_task)

    def create_task_object(self, tid, pkg_id, priority, pickup, dest):
        t = WarehouseTask()
        t.task_id = tid
        t.priority = priority
        t.package_id = pkg_id
        t.pickup_location = pickup
        t.destination_location = dest
        t.status = WarehouseTask.STATUS_PENDING
        t.assigned_robot = ""
        t.pickup_pose = get_location_pose(pickup)
        t.destination_pose = get_location_pose(dest)
        t.created_at = self.get_clock().now().to_msg()
        return t

    def generate_initial_tasks(self, count):
        # Deterministic diverse seed with high coverage
        random.seed(42)
        for i in range(count):
            tid = f"T{self.task_counter:03d}"
            self.task_counter += 1

            pkg_num = ((i % self.package_count) + 1)
            pkg_id = f"P{pkg_num:03d}"

            # Dynamically select priority
            priority = PRIORITY_LEVELS[i % len(PRIORITY_LEVELS)]

            # Dynamically select pickup and destination
            pickup = PICKUP_LOCATIONS[i % len(PICKUP_LOCATIONS)]
            dest = DESTINATION_LOCATIONS[(i * 2 + 1) % len(DESTINATION_LOCATIONS)]

            task = self.create_task_object(tid, pkg_id, priority, pickup, dest)
            self.tasks[tid] = task
            self.get_logger().info(
                f"Generated Task {tid} [{priority}]: Package {pkg_id} from {pickup} -> {dest}"
            )

    def generate_single_task(self):
        tid = f"T{self.task_counter:03d}"
        self.task_counter += 1

        pkg_num = random.randint(1, self.package_count)
        pkg_id = f"P{pkg_num:03d}"
        priority = random.choice(PRIORITY_LEVELS)
        pickup = random.choice(PICKUP_LOCATIONS)
        dest = random.choice(DESTINATION_LOCATIONS)

        task = self.create_task_object(tid, pkg_id, priority, pickup, dest)
        self.tasks[tid] = task
        self.get_logger().info(
            f"Dynamically Added Task {tid} [{priority}]: {pkg_id} from {pickup} -> {dest}"
        )

    def handle_assign_task(self, request, response):
        robot_id = request.robot_id
        task_id = request.task_id

        if task_id not in self.tasks:
            response.success = False
            response.message = f"Task {task_id} not found."
            return response

        task = self.tasks[task_id]
        if task.status != WarehouseTask.STATUS_PENDING and task.assigned_robot != robot_id:
            response.success = False
            response.message = f"Task {task_id} already assigned to {task.assigned_robot} (status: {task.status})."
            return response

        task.assigned_robot = robot_id
        task.status = WarehouseTask.STATUS_ASSIGNED
        response.success = True
        response.message = f"Task {task_id} successfully assigned to Robot {robot_id}."
        self.get_logger().info(response.message)
        return response

    def publish_all_tasks(self):
        summary_lines = [f"=== ACTIVE WAREHOUSE TASKS ({len(self.tasks)}) ==="]
        for tid, task in self.tasks.items():
            self.tasks_pub.publish(task)
            summary_lines.append(
                f"[{task.task_id}] Priority: {task.priority:<8} | Pkg: {task.package_id} | "
                f"From: {task.pickup_location:<12} -> To: {task.destination_location:<14} | "
                f"Status: {task.status:<11} | Robot: {task.assigned_robot or 'UNASSIGNED'}"
            )

        summary_msg = String()
        summary_msg.data = "\n".join(summary_lines)
        self.task_summary_pub.publish(summary_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TaskGeneratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
