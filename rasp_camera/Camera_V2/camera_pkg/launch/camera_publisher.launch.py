from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_pkg',
            executable='camera_publisher',
            name='camera_publisher',
            output='screen',
            parameters=[
                {"camera_index": 0},       
                {"frame_width": 1280},     # 1920 is full HD
                {"frame_height": 720},     # aspect ratio consistent
                {"fps": 30},               # lower if bandwidth is an issue
                {"jpeg_quality": 80},      # 0–100 (higher = better image, more bandwidth)
            ]
        )
    ])
