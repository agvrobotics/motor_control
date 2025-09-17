from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # SLAM Toolbox Online Async Node
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': False,              # True if using Gazebo
                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'base_link',
                'scan_topic': 'scan',               # your lidar topic
                'odom_topic': 'odom',               # your odom topic
                'mode': 'mapping',                  # continuous mapping
                'resolution': 0.003,                 # small cells → smooth map
                'max_laser_range': 10.0,           

                # Fast, frequent updates
                'map_update_interval': 0.02,         # map integration frequency
                'minimum_travel_distance': 0.003,    # update after 1 cm
                'minimum_travel_heading': 0.01,     # update after 0.5°

                # Scan matching improves smoothness
                'use_scan_matching': True,
                'use_scan_matching_odometry': True,

                # Transform publishing
                'transform_publish_period': 0.01,
                'minimum_time_interval': 0.01
            }]
        ),
    ])
