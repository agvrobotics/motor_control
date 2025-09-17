#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Int32MultiArray
import tf2_ros
from math import sin, cos

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')

        # Parameters
        self.wheel_radius = 0.0425   # meters
        self.wheel_separation = 0.218 # meters

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Last encoder counts
        self.last_counts = None  # Will store [FL, FR, RR]

        # Subscriber to encoder counts
        self.sub_enc = self.create_subscription(
            Int32MultiArray, 'encoder_counts', self.encoder_callback, 10
        )

        # Odometry publisher
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Timer to publish odom + tf even if no encoder data
        self.timer = self.create_timer(0.05, self.publish_odom)  # 20 Hz

        self.get_logger().info("Odom Publisher ready, subscribing to encoder_counts")

    def encoder_callback(self, msg: Int32MultiArray):
        counts = msg.data  # [FL, FR, RR]
        if self.last_counts is None:
            self.last_counts = counts
            return

        # Use first two wheels (FL, FR) for differential drive
        delta_left = (counts[0] - self.last_counts[0]) * self.wheel_radius
        delta_right = (counts[1] - self.last_counts[1]) * self.wheel_radius

        self.last_counts = counts

        # Compute delta_s and delta_theta
        delta_s = (delta_right + delta_left) / 2.0
        delta_theta = (delta_right - delta_left) / self.wheel_separation

        # Update robot pose
        self.x += delta_s * cos(self.theta + delta_theta / 2.0)
        self.y += delta_s * sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta

    def publish_odom(self):
        t_now = self.get_clock().now().to_msg()

        # ---------------- Odometry ----------------
        odom = Odometry()
        odom.header.stamp = t_now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Simple quaternion for yaw only
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)

        # ---------------- TF ----------------
        t = TransformStamped()
        t.header.stamp = t_now
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
