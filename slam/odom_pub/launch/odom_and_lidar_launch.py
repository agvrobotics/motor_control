from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # RPLIDAR A1 node
        Node(
            package='rplidar_ros',
            executable='rplidarNode',
            name='rplidar',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'lidar_link_1'
            }]
        ),

        # Your odometry publisher node
        Node(
            package='odom_pub',         # Your package name
            executable='odom_publisher',# Your executable
            name='odom_publisher',
            output='screen'
        )
    ])
