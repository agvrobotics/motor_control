#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import sys
import tty
import termios

class TeleopSerial(Node):
    def __init__(self):
        super().__init__('teleop_serial')

        # Open serial
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.get_logger().info("Connected to Arduino on /dev/ttyACM0")

        # Print instructions
        print("Use keys:")
        print("  w: forward")
        print("  s: reverse")
        print("  space: stop")
        print("  q: quit")

        # Save terminal state
        self.old_settings = termios.tcgetattr(sys.stdin)

        # Timer to check keyboard
        self.timer = self.create_timer(0.1, self.check_key)

    def check_key(self):
        import select
        dr, dw, de = select.select([sys.stdin], [], [], 0)
        if dr:
            key = sys.stdin.read(1)
            if key == 'w':
                self.send_cmd("FORWARD_LOW")
            elif key == 's':
                self.send_cmd("REVERSE")
            elif key == ' ':
                self.send_cmd("STOP")
            elif key == 'q':
                self.cleanup()
                rclpy.shutdown()

    def send_cmd(self, cmd):
        self.ser.write((cmd + '\n').encode())
        self.get_logger().info(f"Sent command: {cmd}")

    def cleanup(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        self.ser.close()

def main(args=None):
    rclpy.init(args=args)
    node = TeleopSerial()

    tty.setcbreak(sys.stdin.fileno())
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()

if __name__ == '__main__':
    main()
