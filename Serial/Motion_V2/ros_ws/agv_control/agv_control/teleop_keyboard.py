#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, tty, termios, select

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.linear_speed = 0.2  # m/s for forward/back
        self.angular_speed = 1.0 # rad/s for turning

        # Save terminal settings
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        self.get_logger().info("Keyboard Teleop Started")
        self.get_logger().info("Use keys: w=forward, s=back, a=left, d=right, space=stop, q=quit")

        self.timer = self.create_timer(0.1, self.check_key)

    def check_key(self):
        dr,dw,de = select.select([sys.stdin], [], [], 0)
        if dr:
            key = sys.stdin.read(1)
            twist = Twist()
            if key == 'w':
                twist.linear.x = self.linear_speed
                twist.angular.z = 0.0
            elif key == 's':
                twist.linear.x = -self.linear_speed
                twist.angular.z = 0.0
            elif key == 'a':
                twist.linear.x = 0.0
                twist.angular.z = self.angular_speed
            elif key == 'd':
                twist.linear.x = 0.0
                twist.angular.z = -self.angular_speed
            elif key == ' ':
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            elif key == 'q':
                self.cleanup()
                rclpy.shutdown()
                return
            else:
                return

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
        pass
    finally:
        node.cleanup()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
