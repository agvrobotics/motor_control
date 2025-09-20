#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import TransformStamped
import tf_transformations
from tf2_ros import TransformBroadcaster
import math

import os

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')

        # Robot parameters
        self.wheel_radius = 0.0425       # meters (4.25 cm)
        self.wheel_separation = 0.232    # meters (23.2 cm between left and right wheels)
        self.counts_per_rev = 4100       # adjust to your encoder
        self.turning_gain = 42
           # skid-steer correction factor (tune!)

        # Initial pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Velocities
        self.v = 0.0
        self.omega = 0.0

        # Encoder state
        self.last_counts = None
        self.last_time = None

        # Subscribers
        self.create_subscription(Int32MultiArray, 'encoder_counts', self.encoder_callback, 10)

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publish at 50Hz
        self.create_timer(0.02, self.publish_odom)

        self.get_logger().info("Odom publisher initialized (using rear RL & RR encoders only)")
        
        # Add this near the top of the class __init__:
        self.log_file = os.path.expanduser("~/odom_omega_log.txt")
        open(self.log_file, "w").close()  

    def encoder_callback(self, msg: Int32MultiArray):
        counts = msg.data  # [RL, RR]
        now_time = self.get_clock().now().nanoseconds * 1e-9  # seconds

        if len(counts) < 2:
            self.get_logger().warn("Encoder message must have [RL, RR], got %s" % str(counts))
            return

        if self.last_counts is None:
            self.last_counts = counts
            self.last_time = now_time
            return

        dt = now_time - self.last_time
        if dt <= 0:
            return

        # Encoder deltas (ignore FR, use RL and RR)
        delta_RL = counts[0] - self.last_counts[0]
        delta_RR = counts[1] - self.last_counts[1]

        # Convert encoder counts to meters
        dist_RL = 2 * math.pi * self.wheel_radius * delta_RL / self.counts_per_rev
        dist_RR = 2 * math.pi * self.wheel_radius * delta_RR / self.counts_per_rev

        # Left vs right distances
        left_dist = dist_RL
        right_dist = dist_RR

        # Replace the print/info section with:
        omega_raw = (right_dist - left_dist) / self.wheel_separation
        with open(self.log_file, "a") as f:
            f.write(f"{omega_raw:.4f},{omega_raw * self.turning_gain:.4f}\n")      
        # Velocities
        self.v = (left_dist + right_dist) / (2.0 * dt)
        self.omega = ((right_dist - left_dist) / self.wheel_separation) * self.turning_gain

        # Pose update
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta += self.omega * dt

        # Save state
        self.last_counts = counts
        self.last_time = now_time

    def publish_odom(self):
        now_time = self.get_clock().now()

        # TF
        t = TransformStamped()
        t.header.stamp = now_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        quat = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(t)

        # Odometry
        odom = Odometry()
        odom.header.stamp = now_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]

        odom.twist.twist.linear.x = self.v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.omega

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
