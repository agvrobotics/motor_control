from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_pkg',
            executable='camera_subscriber',
            name='camera_subscriber',
            output='screen',
            parameters=[
                {"roi_size": 150}, 
                {"color_threshold": 500},  
                {"show_image": True},       
            ]
        )
    ])
