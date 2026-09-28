#!/usr/bin/env python3
"""
Master Launch File for 3-AMR Edge Fleet Coordination.
Supports Dual-Mode execution:
  - mode:=EDGE_AI       (Proposed: Decentralized ORCA velocity negotiation)
  - mode:=STOP_AND_WAIT (Baseline: Traditional stop-and-wait benchmark)

SIH Problem Statement 26123 (Bharat Electronics Limited)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='EDGE_AI',
        description='Coordination mode: Decentralized EDGE_AI'
    )
    mode = LaunchConfiguration('mode')

    # AMR 1: Crosses central intersection (5.0, 5.0) from West to East
    robot_1_nodes = GroupAction([
        PushRosNamespace('robot_1'),
        Node(
            package='edge_fleet_bringup',
            executable='simulated_amr.py',
            name='simulated_amr',
            parameters=[{
                'robot_id': 'robot_1',
                'start_x': 0.0,
                'start_y': 5.0,
                'start_yaw': 0.0,
                'max_speed': 0.7,
                'route_waypoints': '0.0,5.0; 10.0,5.0; 10.0,9.0; 0.0,9.0; 0.0,5.0'
            }],
            output='screen'
        ),
        Node(
            package='edge_fleet_core',
            executable='p2p_peer_node',
            name='p2p_peer_node',
            parameters=[{'robot_id': 'robot_1', 'priority_level': 7}],
            output='screen'
        ),
        Node(
            package='edge_fleet_core',
            executable='conflict_resolver_node',
            name='conflict_resolver_node',
            parameters=[{
                'robot_id': 'robot_1',
                'mode': mode,
                'priority_level': 7,
                'safety_margin': 0.25,
                'time_horizon': 4.0
            }],
            output='screen'
        ),
        Node(
            package='edge_fleet_core',
            executable='task_allocator_node',
            name='task_allocator_node',
            parameters=[{'robot_id': 'robot_1'}],
            output='screen'
        ),
        Node(
            package='edge_fleet_core',
            executable='edge_perception_node',
            name='edge_perception_node',
            parameters=[{'robot_id': 'robot_1'}],
            output='screen'
        )
    ])

    # AMR 2: Crosses central intersection (5.0, 5.0) from South to North (Perpendicular to AMR 1)
    robot_2_nodes = GroupAction([
        PushRosNamespace('robot_2'),
        Node(
            package='edge_fleet_bringup',
            executable='simulated_amr.py',
            name='simulated_amr',
            parameters=[{
                'robot_id': 'robot_2',
                'start_x': 5.0,
                'start_y': 0.0,
                'start_yaw': 1.57,
                'max_speed': 0.7,
                'route_waypoints': '5.0,0.0; 5.0,10.0; 1.0,10.0; 1.0,0.0; 5.0,0.0'
            }],
            output='screen'
        ),
        Node(
            package='edge_fleet_core',
            executable='p2p_peer_node',
            name='p2p_peer_node',
            parameters=[{'robot_id': 'robot_2', 'priority_level': 4}],
            output='screen'
        ),
        Node(
            package='edge_fleet_core',
            executable='conflict_resolver_node',
            name='conflict_resolver_node',
            parameters=[{
                'robot_id': 'robot_2',
                'mode': mode,
                'priority_level': 4,
                'safety_margin': 0.25,
                'time_horizon': 4.0
            }],
            output='screen'
        ),
        Node(
            package='edge_fleet_core',
            executable='task_allocator_node',
            name='task_allocator_node',
            parameters=[{'robot_id': 'robot_2'}],
            output='screen'
        ),
        Node(
            package='edge_fleet_core',
            executable='edge_perception_node',
            name='edge_perception_node',
            parameters=[{'robot_id': 'robot_2'}],
            output='screen'
        )
    ])

    # AMR 3: Operates in diagonal warehouse corridor
    robot_3_nodes = GroupAction([
        PushRosNamespace('robot_3'),
        Node(
            package='edge_fleet_bringup',
            executable='simulated_amr.py',
            name='simulated_amr',
            parameters=[{
                'robot_id': 'robot_3',
                'start_x': 8.5,
                'start_y': 2.0,
                'start_yaw': 3.14,
                'max_speed': 0.75,
                'route_waypoints': '8.5,2.0; 2.5,2.0; 2.5,7.0; 8.5,7.0; 8.5,2.0'
            }],
            output='screen'
        ),
        Node(
            package='edge_fleet_core',
            executable='p2p_peer_node',
            name='p2p_peer_node',
            parameters=[{'robot_id': 'robot_3', 'priority_level': 5}],
            output='screen'
        ),
        Node(
            package='edge_fleet_core',
            executable='conflict_resolver_node',
            name='conflict_resolver_node',
            parameters=[{
                'robot_id': 'robot_3',
                'mode': mode,
                'priority_level': 5,
                'safety_margin': 0.25,
                'time_horizon': 4.0
            }],
            output='screen'
        ),
        Node(
            package='edge_fleet_core',
            executable='task_allocator_node',
            name='task_allocator_node',
            parameters=[{'robot_id': 'robot_3'}],
            output='screen'
        ),
        Node(
            package='edge_fleet_core',
            executable='edge_perception_node',
            name='edge_perception_node',
            parameters=[{'robot_id': 'robot_3'}],
            output='screen'
        )
    ])

    # Global Benchmark Evaluator (Tracks zero collisions and 20%+ efficiency metric)
    benchmark_node = Node(
        package='edge_fleet_core',
        executable='fleet_benchmark_node',
        name='fleet_benchmark_node',
        parameters=[{
            'active_mode': mode,
            'collision_dist_threshold': 0.40
        }],
        output='screen'
    )

    # Passive Web & WebSocket Fleet Dashboard (Runs on http://localhost:8080)
    dashboard_node = Node(
        package='edge_fleet_bringup',
        executable='dashboard_server.py',
        name='fleet_dashboard_node',
        output='screen'
    )

    return LaunchDescription([
        mode_arg,
        robot_1_nodes,
        robot_2_nodes,
        robot_3_nodes,
        benchmark_node,
        dashboard_node
    ])
