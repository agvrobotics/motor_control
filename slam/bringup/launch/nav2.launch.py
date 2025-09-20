from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    bringup_launch = os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")

    map_file = "/home/sierra-95/Documents/robodojo/slam/michael.yaml"

    return LaunchDescription([
        # Map server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            launch_arguments={
                "map": map_file,
                "use_sim_time": "false"
            }.items(),
        ),
    ])
