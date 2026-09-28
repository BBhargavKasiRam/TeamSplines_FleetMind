import os
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

ROBOT_FLEET = [
    {"name": "Alpha", "color": "Blue", "ns": "robot_alpha", "id": 1, "x": -12.5, "y": -2.0, "z": 0.1, "yaw": 0.0},
    {"name": "Beta", "color": "Red", "ns": "robot_beta", "id": 2, "x": -12.5, "y": -1.0, "z": 0.1, "yaw": 0.0},
    {"name": "Gamma", "color": "Green", "ns": "robot_gamma", "id": 3, "x": -12.5, "y": 0.0, "z": 0.1, "yaw": 0.0},
    {"name": "Delta", "color": "Yellow", "ns": "robot_delta", "id": 4, "x": -12.5, "y": 1.0, "z": 0.1, "yaw": 0.0},
    {"name": "Epsilon", "color": "Purple", "ns": "robot_epsilon", "id": 5, "x": -12.5, "y": 2.0, "z": 0.1, "yaw": 0.0},
]

def launch_setup(context, *args, **kwargs):
    robot_count_str = LaunchConfiguration('robot_count').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    try:
        robot_count = int(robot_count_str)
    except ValueError:
        robot_count = 5

    desc_share = get_package_share_directory('warehouse_3d_description')
    xacro_file = os.path.join(desc_share, 'urdf', 'warehouse_amr.urdf.xacro')

    actions = []
    active_fleet = ROBOT_FLEET[:max(1, min(robot_count, len(ROBOT_FLEET)))]

    for rob in active_fleet:
        name = rob['name']
        color = rob['color']
        ns = rob['ns']
        rid = rob['id']

        # Process xacro
        res = subprocess.run([
            'xacro', xacro_file,
            f'robot_name:={name}',
            f'robot_color:={color}',
            f'robot_namespace:={ns}',
            f'robot_id:={rid}'
        ], capture_output=True, text=True)

        urdf_content = res.stdout

        # Robot State Publisher for each robot with namespace
        rsp_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=ns,
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': urdf_content,
                'use_sim_time': use_sim_time,
                'frame_prefix': f"{ns}/"
            }]
        )
        actions.append(rsp_node)

    # Robot State Broadcaster Node
    broadcaster_node = Node(
        package='warehouse_3d_robot',
        executable='robot_state_broadcaster',
        name='robot_state_broadcaster',
        output='screen',
        parameters=[{
            'robot_count': robot_count,
            'use_sim_time': use_sim_time
        }]
    )
    actions.append(broadcaster_node)

    return actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_count', default_value='5', description='Number of AMRs'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        OpaqueFunction(function=launch_setup)
    ])
