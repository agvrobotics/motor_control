from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    # Spawner for Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    # Spawner for Diff Drive Controller
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    return LaunchDescription([
        joint_state_broadcaster_spawner,
        diff_drive_spawner
    ])
