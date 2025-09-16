import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sierra-95/Documents/motor_control/Serial/Motion_V1/ros_ws/install/teleop_serial'
