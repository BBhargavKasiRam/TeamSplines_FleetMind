import os
import subprocess
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
    OpaqueFunction,
    TimerAction
)
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
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
    package_count_str = LaunchConfiguration('package_count').perform(context)
    task_count_str = LaunchConfiguration('task_count').perform(context)
    use_rviz = LaunchConfiguration('use_rviz')
    use_mock = LaunchConfiguration('use_mock_controller')
    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')

    try:
        robot_count = int(robot_count_str)
        package_count = int(package_count_str)
        task_count = int(task_count_str)
    except ValueError:
        robot_count, package_count, task_count = 5, 30, 10

    # Package directories
    bringup_share = get_package_share_directory('warehouse_3d_bringup')
    world_share = get_package_share_directory('warehouse_3d_world')
    desc_share = get_package_share_directory('warehouse_3d_description')
    vis_share = get_package_share_directory('warehouse_3d_visualization')

    world_sdf_path = os.path.join(world_share, 'worlds', 'warehouse.sdf')
    bridge_config_path = os.path.join(bringup_share, 'config', 'ros_gz_bridge.yaml')
    rviz_config_path = os.path.join(vis_share, 'config', 'warehouse_3d.rviz')
    xacro_file = os.path.join(desc_share, 'urdf', 'warehouse_amr.urdf.xacro')

    actions = []

    # 1. Gazebo Sim Process
    gz_sim_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_sdf_path],
        output='screen',
        condition=UnlessCondition(headless),
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '1', 'QT_X11_NO_MITSHM': '1'}
    )
    gz_sim_headless = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', world_sdf_path],
        output='screen',
        condition=IfCondition(headless)
    )
    actions.extend([gz_sim_gui, gz_sim_headless])

    # 2. ROS-GZ Parameter Bridge
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_config_path,
            'use_sim_time': use_sim_time
        }]
    )
    # Delay bridge slightly to let Gazebo start
    actions.append(TimerAction(period=2.0, actions=[bridge_node]))

    # 3. Setup Robots
    active_fleet = ROBOT_FLEET[:max(1, min(robot_count, len(ROBOT_FLEET)))]
    spawn_actions = []

    for rob in active_fleet:
        name = rob['name']
        color = rob['color']
        ns = rob['ns']
        rid = rob['id']
        x = rob['x']
        y = rob['y']
        z = rob['z']
        yaw = rob['yaw']

        res = subprocess.run([
            'xacro', xacro_file,
            f'robot_name:={name}',
            f'robot_color:={color}',
            f'robot_namespace:={ns}',
            f'robot_id:={rid}'
        ], capture_output=True, text=True)
        urdf_content = res.stdout

        # Robot State Publisher
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



    # 4. Robot State Broadcaster (TF + RobotState)
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

    # 5. Dynamic Task Generator
    task_gen_node = Node(
        package='warehouse_task_generator',
        executable='task_generator_node',
        name='warehouse_task_generator',
        output='screen',
        parameters=[{
            'robot_count': robot_count,
            'package_count': package_count,
            'task_count': task_count,
            'continuous_generation': False,
            'generation_interval': 10.0,
            'use_sim_time': use_sim_time
        }]
    )
    actions.append(task_gen_node)

    # 6. Logic Adapter
    adapter_node = Node(
        package='warehouse_logic_adapter',
        executable='logic_adapter_node',
        name='warehouse_logic_adapter',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    actions.append(adapter_node)

    # 7. Mock Controller (Optional for testing)
    mock_ctrl_node = Node(
        package='warehouse_logic_adapter',
        executable='mock_controller',
        name='mock_controller',
        output='screen',
        condition=IfCondition(use_mock),
        parameters=[{'use_sim_time': use_sim_time}]
    )
    actions.append(mock_ctrl_node)

    # 8. 3D RViz Marker Publisher
    marker_node = Node(
        package='warehouse_3d_visualization',
        executable='warehouse_marker_publisher',
        name='warehouse_marker_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    actions.append(marker_node)

    # 9. RViz2 GUI
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': use_sim_time}],
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'}
    )
    actions.append(rviz_node)

    return actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_count', default_value='5', description='Number of active warehouse robots (up to 5)'),
        DeclareLaunchArgument('package_count', default_value='30', description='Number of cardboard package boxes'),
        DeclareLaunchArgument('task_count', default_value='10', description='Initial number of simultaneous dynamic tasks'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Launch RViz2 visualization'),
        DeclareLaunchArgument('use_mock_controller', default_value='true', description='Enable mock controller for testing'),
        DeclareLaunchArgument('headless', default_value='false', description='Run Gazebo in headless mode (server only)'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use Gazebo simulation time'),
        OpaqueFunction(function=launch_setup)
    ])
