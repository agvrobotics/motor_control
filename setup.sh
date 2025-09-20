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
ros2 launch bringup odom.launch.py

cd ~/slam1/ros2_ws

tmux new -s rplidar
source install/setup.bash
ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/ttyUSB0 serial_baudrate:=115200 frame_id:=lidar_link_1


##################On PC###########################
cd ~/Documents/robodojo/slam
source install/setup.bash
ros2 launch dekut_amr_description display.launch.py
ros2 launch bringup slam.launch.py

##--------------confirm tree structure----------------##
ros2 run tf2_tools view_frames

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
ros2 launch bringup nav2.launch.py