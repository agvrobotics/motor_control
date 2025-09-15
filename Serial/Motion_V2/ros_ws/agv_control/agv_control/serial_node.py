#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import serial
import threading

class SerialCmdNode(Node):
    def __init__(self, ser):
        super().__init__('serial_cmd_node')
        self.ser = ser
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10
        )
        self.get_logger().info("SerialCmdNode ready")

    def cmd_vel_callback(self, msg: Twist):
        linear = msg.linear.x
        angular = msg.angular.z
        cmd_str = f"{linear},{angular}\n"
        self.get_logger().info(f"TX -> {cmd_str.strip()}")
        self.ser.write(cmd_str.encode('utf-8'))


class SerialFeedbackNode(Node):
    def __init__(self, ser):
        super().__init__('serial_feedback_node')
        self.ser = ser
        self.pub = self.create_publisher(String, 'encoder_counts', 10)

        # Start background reader
        self.thread = threading.Thread(target=self.read_loop, daemon=True)
        self.thread.start()

        self.get_logger().info("SerialFeedbackNode ready")

    def read_loop(self):
        with open("encoder_log.csv", "w") as f:
            f.write("FR,RL,RR\n")
            while rclpy.ok():
                try:
                    line = self.ser.readline().decode('utf-8').strip()
                    if line:
                        self.get_logger().info(f"RX <- {line}")
                        # publish as string
                        msg = String()
                        msg.data = line
                        self.pub.publish(msg)

                        # log to file
                        f.write(line + "\n")
                        f.flush()
                except Exception as e:
                    self.get_logger().warn(f"Serial read error: {e}")


def main(args=None):
    rclpy.init(args=args)

    # Open serial only once, share between nodes
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)

    cmd_node = SerialCmdNode(ser)
    feedback_node = SerialFeedbackNode(ser)

    # Spin both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(cmd_node)
    executor.add_node(feedback_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        cmd_node.destroy_node()
        feedback_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
