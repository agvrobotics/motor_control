from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Map server
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[{"yaml_filename": "/home/agv/maps/michael.yaml"}]
        ),

        # AMCL localization
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "odom_frame_id": "odom",
                "base_frame_id": "base_link",
                "scan_topic": "scan"
            }]
        ),

        # Nav2 bringup
        Node(
            package="nav2_bringup",
            executable="bringup_launch.py",
            output="screen",
            parameters=[
                {"use_sim_time": False},
                "/home/agv/robodojo/slam/lidar_nav2/params/nav2_params.yaml"
            ]
        ),
    ])
