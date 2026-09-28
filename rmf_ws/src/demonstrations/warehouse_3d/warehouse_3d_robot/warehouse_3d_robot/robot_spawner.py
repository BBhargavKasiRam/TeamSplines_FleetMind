#!/usr/bin/env python3
"""
Robot Spawner Node
Handles dynamic fleet configuration and spawning of AMR robots into Gazebo Sim.
"""

import os
import subprocess
import time
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

ROBOT_FLEET = [
    {"name": "Alpha", "color": "Blue", "ns": "robot_alpha", "id": 1, "x": -12.5, "y": -2.0, "z": 0.1, "yaw": 0.0},
    {"name": "Beta", "color": "Red", "ns": "robot_beta", "id": 2, "x": -12.5, "y": -1.0, "z": 0.1, "yaw": 0.0},
    {"name": "Gamma", "color": "Green", "ns": "robot_gamma", "id": 3, "x": -12.5, "y": 0.0, "z": 0.1, "yaw": 0.0},
    {"name": "Delta", "color": "Yellow", "ns": "robot_delta", "id": 4, "x": -12.5, "y": 1.0, "z": 0.1, "yaw": 0.0},
    {"name": "Epsilon", "color": "Purple", "ns": "robot_epsilon", "id": 5, "x": -12.5, "y": 2.0, "z": 0.1, "yaw": 0.0},
]

class RobotSpawner(Node):
    def __init__(self):
        super().__init__('robot_spawner')
        self.declare_parameter('robot_count', 5)
        self.robot_count = self.get_parameter('robot_count').get_parameter_value().integer_value
        self.get_logger().info(f"Initializing Robot Spawner for {self.robot_count} robots...")

        # Locate Xacro file
        desc_share = get_package_share_directory('warehouse_3d_description')
        self.xacro_path = os.path.join(desc_share, 'urdf', 'warehouse_amr.urdf.xacro')

        self.spawn_fleet()

    def spawn_fleet(self):
        fleet_to_spawn = ROBOT_FLEET[:max(1, min(self.robot_count, len(ROBOT_FLEET)))]
        for rob in fleet_to_spawn:
            self.spawn_robot(rob)
            time.sleep(0.5)
        self.get_logger().info(f"Successfully spawned {len(fleet_to_spawn)} robots.")

    def spawn_robot(self, rob_info):
        ns = rob_info['ns']
        name = rob_info['name']
        color = rob_info['color']
        rid = rob_info['id']
        x = rob_info['x']
        y = rob_info['y']
        z = rob_info['z']
        yaw = rob_info['yaw']

        self.get_logger().info(f"Generating URDF and spawning Robot {name} ({color}) in /{ns}...")

        # Process xacro
        cmd_xacro = [
            'xacro', self.xacro_path,
            f'robot_name:={name}',
            f'robot_color:={color}',
            f'robot_namespace:={ns}',
            f'robot_id:={rid}'
        ]
        res = subprocess.run(cmd_xacro, capture_output=True, text=True)
        if res.returncode != 0:
            self.get_logger().error(f"Xacro error for {name}: {res.stderr}")
            return

        urdf_content = res.stdout
        tmp_urdf = f"/tmp/{ns}.urdf"
        with open(tmp_urdf, "w") as f:
            f.write(urdf_content)

        # Call ros_gz_sim create
        cmd_create = [
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-name', f"amr_{ns}",
            '-file', tmp_urdf,
            '-x', str(x), '-y', str(y), '-z', str(z), '-Y', str(yaw)
        ]
        sp_res = subprocess.run(cmd_create, capture_output=True, text=True)
        if sp_res.returncode == 0:
            self.get_logger().info(f"Spawned {name} at ({x}, {y}, {z})")
        else:
            self.get_logger().warn(f"Spawn result for {name}: {sp_res.stdout} {sp_res.stderr}")

def main(args=None):
    rclpy.init(args=args)
    spawner = RobotSpawner()
    # Spawner runs and finishes
    rclpy.shutdown()

if __name__ == '__main__':
    main()
