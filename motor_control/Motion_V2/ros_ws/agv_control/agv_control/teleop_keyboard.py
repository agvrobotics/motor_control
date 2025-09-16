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

        # Save terminal settings
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        self.get_logger().info("Keyboard Teleop Started")
        self.get_logger().info("Use keys: w=forward, s=back, a=left, d=right, space=stop, q=quit")

        self.timer = self.create_timer(0.1, self.check_key)

    
    def check_key(self):
        dr, _, _ = select.select([sys.stdin], [], [], 0)
        if dr:
            key = sys.stdin.read(1)
            twist = Twist()

            if key == ' ':
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            # Quit
            elif key == 'q':
                self.stop_robot()
                self.cleanup()
                rclpy.shutdown()
                return

            else:
                linear = 0.0
                angular = 0.0

                if key == 'w':
                    linear = self.linear_speed
                elif key == 's':
                    linear = -self.linear_speed

                if key == 'a':
                    angular = self.angular_speed
                elif key == 'd':
                    angular = -self.angular_speed

                twist.linear.x = linear
                twist.angular.z = angular

            self.pub.publish(twist)

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
