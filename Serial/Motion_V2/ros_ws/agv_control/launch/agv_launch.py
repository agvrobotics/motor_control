from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='agv_control',
            executable='serial_node',
            name='serial_node',
            output='screen',
            parameters=[{
               'port': '/dev/ttyACM0',
                'baudrate': 115200
            }]
        ),
    ])
