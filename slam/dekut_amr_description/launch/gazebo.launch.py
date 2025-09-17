from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    # Get package share directory
    share_dir = get_package_share_directory('dekut_amr_description')

    # Process XACRO/URDF
    urdf_file = os.path.join(share_dir, 'urdf', 'dekut_amr.xacro')
    robot_description_config = xacro.process_file(urdf_file)
    robot_urdf = robot_description_config.toxml()

    # Robot State Publisher (for TFs)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_urdf}]
    )

    # Joint State Publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # Launch Ignition Gazebo (Fortress)
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-v', '4', '-r'],  # verbose 4, real-time
        output='screen'
    )

    # Spawn the robot directly from URDF string
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-string', robot_urdf,
            '-name', 'dekut_amr'
        ]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher_node,
        joint_state_publisher_node,
        spawn_entity
    ])
