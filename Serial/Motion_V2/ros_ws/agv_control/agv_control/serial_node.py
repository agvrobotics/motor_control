#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32MultiArray
import serial
import threading


class SerialManager(Node):
    def __init__(self):
        super().__init__('serial_manager')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        self.ser = serial.Serial(port, baudrate, timeout=0.1)
        self.get_logger().info(f"Opened serial at {port} {baudrate}bps")

class SerialCmdNode(Node):
    def __init__(self, ser):
        super().__init__('serial_cmd_node')
        self.ser = ser
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10
        )
        self.get_logger().info("Serial cmd_vel ready")

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
        self.pub_raw = self.create_publisher(String, 'encoder_counts_raw', 10)
        self.pub_enc = self.create_publisher(Int32MultiArray, 'encoder_counts', 10)

        # Start background reader
        self.thread = threading.Thread(target=self.read_loop, daemon=True)
        self.thread.start()

        self.get_logger().info("Serial feedback ready")

    def read_loop(self):
        while rclpy.ok():
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if not line:
                    continue

                self.get_logger().info(f"RX <- {line}")

                # Publish raw string - "123,456,789"
                msg_raw = String()
                msg_raw.data = line
                self.pub_raw.publish(msg_raw)

                # Publish structured encoder values - [123, 456, 789]
                parts = line.split(',')
                if len(parts) == 3:
                    try:
                        counts = [int(parts[0]), int(parts[1]), int(parts[2])]
                        msg_enc = Int32MultiArray()
                        msg_enc.data = counts
                        self.pub_enc.publish(msg_enc)
                    except ValueError:
                        self.get_logger().warn(f"Bad encoder data: {line}")

            except Exception as e:
                self.get_logger().warn(f"Serial read error: {e}")

def main(args=None):
    rclpy.init(args=args)

    manager = SerialManager()
    cmd_node = SerialCmdNode(manager.ser)
    feedback_node = SerialFeedbackNode(manager.ser)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(manager)
    executor.add_node(cmd_node)
    executor.add_node(feedback_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        manager.ser.close()
        cmd_node.destroy_node()
        feedback_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
