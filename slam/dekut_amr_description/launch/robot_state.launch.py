import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro

def generate_launch_description():

    # =========================
    # Launch arguments
    # =========================
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration(
        'world_file', 
        default=os.path.expanduser('~/ignition_models/dekut_amr/empty_world.sdf')
    )

    # =========================
    # URDF/XACRO processing
    # =========================
    pkg_path = get_package_share_directory('dekut_amr_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'dekut_amr.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml(),
                         'use_sim_time': use_sim_time}

    # =========================
    # Ignition Gazebo process
    # =========================
    ign_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', world_file, '-v4'],
        output='screen'
    )

    # =========================
    # Robot State Publisher
    # =========================
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # =========================
    # Controller manager node
    # =========================
    diff_drive_yaml = os.path.join(pkg_path, 'config', 'diff_drive.yaml')
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        # Load YAML first, then robot_description
        parameters=[diff_drive_yaml, robot_description]
    )

    # =========================
    # Load and activate diff_drive_controller
    # Delayed by 5 seconds to ensure controller_manager is up
    # =========================
    load_diff_drive = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'diff_drive_controller'],
                output='screen'
            )
        ]
    )

    # =========================
    # Launch Description
    # =========================
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time'),
        DeclareLaunchArgument(
            'world_file', 
            default_value=os.path.expanduser('~/ignition_models/dekut_amr/empty_world.sdf'), 
            description='Path to SDF world'
        ),
        ign_gazebo,
        robot_state_publisher_node,
        ros2_control_node,
        load_diff_drive
    ])
