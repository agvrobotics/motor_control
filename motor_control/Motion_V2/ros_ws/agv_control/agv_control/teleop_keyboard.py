#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, tty, termios, select

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.linear_speed = 0.2  # m/s
        self.angular_speed = 1.0  # rad/s

        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        self.get_logger().info("Keyboard Teleop Started")
        self.get_logger().info("Use keys: w=forward, s=back, a=left, d=right, space=stop, q=quit")

        self.timer = self.create_timer(0.1, self.check_key)
        self.active_key = None

    
    def check_key(self):
        dr, _, _ = select.select([sys.stdin], [], [], 0)
        if dr:
            key = sys.stdin.read(1)
            if key == '\x1b': 
                key2 = sys.stdin.read(1)
                key3 = sys.stdin.read(1)
                seq = key + key2 + key3
                self.active_key = seq
            else:
                if key == 'q':
                    self.stop_robot()
                    self.cleanup()
                    rclpy.shutdown()
                    self.destroy_node()
                    sys.exit(0)
                else:
                    self.active_key = key

        linear, angular = self.key_to_twist(self.active_key)
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.pub.publish(twist)

    def key_to_twist(self, key):
        if key == '\x1b[A':   # Up
            return self.linear_speed, 0.0
        elif key == '\x1b[B': # Down
            return -self.linear_speed, 0.0
        elif key == '\x1b[D': # Left
            return 0.0, self.angular_speed
        elif key == '\x1b[C': # Right
            return 0.0, -self.angular_speed
        elif key == '[':      # Arc left
            return self.linear_speed, self.angular_speed * 0.5
        elif key == ']':      # Arc right
            return self.linear_speed, -self.angular_speed * 0.5
        elif key == ' ':      # Emergency stop
            return 0.0, 0.0
        else:
            return 0.0, 0.0



    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.pub.publish(twist)

    def cleanup(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        self.get_logger().info("Exiting Keyboard Teleop")

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        node.cleanup()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
