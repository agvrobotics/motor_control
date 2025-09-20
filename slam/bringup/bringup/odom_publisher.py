#!/usr/bin/env python3
"""
Odometry publisher for ROS2 (Humble).
- Uses rear-left and rear-right encoder counts (Int32MultiArray [RL, RR]).
- Publishes nav_msgs/msg/Odometry on topic 'odom'
- Broadcasts TF odom -> base_link
- Default publish rate: 20 Hz
"""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import tf_transformations


class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')

        # --- Parameters ---
        self.declare_parameter('wheel_radius', 0.0425)
        self.declare_parameter('wheel_separation', 0.232)
        self.declare_parameter('counts_per_rev', 4100)
        self.declare_parameter('turning_gain', 1.0)
        self.declare_parameter('publish_hz', 20.0)
        self.declare_parameter('smoothing_alpha', 0.3)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('pose_cov_xy', 0.02)
        self.declare_parameter('pose_cov_yaw', 0.05)
        self.declare_parameter('twist_cov_v', 0.02)
        self.declare_parameter('twist_cov_omega', 0.02)

        # load params
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.counts_per_rev = self.get_parameter('counts_per_rev').value
        self.turning_gain = self.get_parameter('turning_gain').value
        self.publish_hz = self.get_parameter('publish_hz').value
        self.alpha = self.get_parameter('smoothing_alpha').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.pose_cov_xy = self.get_parameter('pose_cov_xy').value
        self.pose_cov_yaw = self.get_parameter('pose_cov_yaw').value
        self.twist_cov_v = self.get_parameter('twist_cov_v').value
        self.twist_cov_omega = self.get_parameter('twist_cov_omega').value

        # --- Internal state ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.omega = 0.0
        self.last_counts = None
        self.last_time = None
        self.ticks_to_m = (2.0 * math.pi * self.wheel_radius) / self.counts_per_rev

        # Publisher / Broadcaster / Subscriber
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Int32MultiArray, 'encoder_counts', self.encoder_callback, 10)

        # Timer
        period = 1.0 / max(1.0, self.publish_hz)
        self.create_timer(period, self.publish_odom)
        self.get_logger().info(f"OdomPublisher ready: {self.odom_frame} -> {self.base_frame} @ {self.publish_hz} Hz")

    def encoder_callback(self, msg: Int32MultiArray):
        counts = msg.data
        if len(counts) < 2:
            self.get_logger().warn("encoder_counts must be [RL, RR]")
            return

        now = self.get_clock().now()

        if self.last_counts is None:
            self.last_counts = [int(counts[0]), int(counts[1])]
            self.last_time = now
            return

        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return

        delta_RL = int(counts[0]) - int(self.last_counts[0])
        delta_RR = int(counts[1]) - int(self.last_counts[1])

        dist_RL = delta_RL * self.ticks_to_m
        dist_RR = delta_RR * self.ticks_to_m

        raw_v = (dist_RL + dist_RR) / (2.0 * dt)
        raw_omega = (dist_RR - dist_RL) / (self.wheel_separation * dt)
        omega_scaled = raw_omega * self.turning_gain

        # EMA smoothing
        self.v = (self.alpha * raw_v) + ((1.0 - self.alpha) * self.v)
        self.omega = (self.alpha * omega_scaled) + ((1.0 - self.alpha) * self.omega)

        # integrate pose
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta = (self.theta + self.omega * dt + math.pi) % (2.0 * math.pi) - math.pi

        self.last_counts = [int(counts[0]), int(counts[1])]
        self.last_time = now

    def publish_odom(self):
        now = self.get_clock().now()

        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        quat = tf_transformations.quaternion_from_euler(0.0, 0.0, self.theta)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]

        pose_cov = [0.0] * 36
        pose_cov[0] = self.pose_cov_xy
        pose_cov[7] = self.pose_cov_xy
        pose_cov[35] = self.pose_cov_yaw
        odom.pose.covariance = pose_cov

        odom.twist.twist.linear.x = self.v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.omega

        twist_cov = [0.0] * 36
        twist_cov[0] = self.twist_cov_v
        twist_cov[35] = self.twist_cov_omega
        odom.twist.covariance = twist_cov

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


if __name__ == '__main__':
    main()
