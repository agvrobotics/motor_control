######################on Raspberry PI###########################
cd ~/robodojo/motor_control/Motion_V2/ros_ws

tmux new -s serial
source install/setup.bash
ros2 launch agv_control agv_launch.py

tmux new -s teleop
source install/setup.bash
ros2 run agv_control keyboard_teleop


cd ~/robodojo/slam
#encoder info publisher(subscribes to serial)
tmux new -s odom
source install/setup.bash
ros2 launch odom_pub odom_pub.launch.py

cd ~/slam1/ros2_ws

tmux new -s rplidar
source install/setup.bash
ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/ttyUSB0 serial_baudrate:=115200 frame_id:=lidar_link_1


##################On PC###########################
cd ~/Documents/robodojo/slam
source install/setup.bash
ros2 launch dekut_amr_description display.launch.py
ros2 launch lidar_slam slam.launch.py

##--------------confirm tree structure----------------##
ros2 run tf2_tools view_frames.py

#on Rviz
laser scan topic: /scan
map topic: /map
odometry topic: /odom

fixed frame: map
views: top_down


#### NAV2 #########################
cd ~/robodojo/slam

tmux new -s nav2
source install/setup.bash
ros2 launch lidar_nav2 nav2.launch.py

ros2 launch nav2_bringup navigation_launch.py
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/home/sierra-95/Documents/robodojo/slam/michael.yaml
