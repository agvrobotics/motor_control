#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, tty, termios, select

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.linear_speed = 0.2   # m/s
        self.angular_speed = 1.0  # rad/s

        # Save terminal settings
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        self.get_logger().info("Keyboard Teleop Started")
        self.get_logger().info("Use arrows or w/a/s/d for motion")
        self.get_logger().info("[ / ] = arc left/right, space = stop, q = quit")

        self.timer = self.create_timer(0.05, self.check_key)  # 20 Hz update
        self.active_key = None  # remember currently pressed key

    def check_key(self):
        dr, _, _ = select.select([sys.stdin], [], [], 0)

        if dr:  # a key was pressed
            key = sys.stdin.read(1)
            if key == '\x1b':  # arrow keys
                key2 = sys.stdin.read(1)
                key3 = sys.stdin.read(1)
                self.active_key = key + key2 + key3
            else:
                if key == 'q':  # quit
                    self.stop_robot()
                    self.cleanup()
                    self.destroy_node()
                    rclpy.shutdown()
                    sys.exit(0)
                elif key == ' ':  # emergency stop
                    self.active_key = None
                else:
                    self.active_key = key

        # use remembered key (keeps moving while held)
        linear, angular = self.key_to_twist(self.active_key)

        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.pub.publish(twist)

    def key_to_twist(self, key):
        if key == '\x1b[A' or key == 'w':   # Up
            return self.linear_speed, 0.0
        elif key == '\x1b[B' or key == 's': # Down
            return -self.linear_speed, 0.0
        elif key == '\x1b[D' or key == 'a': # Left
            return 0.0, self.angular_speed
        elif key == '\x1b[C' or key == 'd': # Right
            return 0.0, -self.angular_speed
        elif key == '[':  # Arc left
            return self.linear_speed, self.angular_speed * 0.5
        elif key == ']':  # Arc right
            return self.linear_speed, -self.angular_speed * 0.5
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
