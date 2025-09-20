#!/usr/bin/env python3
"""
Odometry publisher for ROS2 (Humble).
- Uses rear-left and rear-right encoder counts (Int32MultiArray [RL, RR]).
- Publishes nav_msgs/msg/Odometry on topic 'odom'
- Broadcasts TF odom -> base_link
- Default publish rate: 20 Hz (adjust via parameter)
- Applies small EMA smoothing to velocities to reduce jitter
- Provides configurable covariances
"""
import os
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

        # --- Parameters (tweak from launch or ros2 param set) ---
        self.declare_parameter('wheel_radius', 0.0425)        # meters
        self.declare_parameter('wheel_separation', 0.232)     # meters (track width)
        self.declare_parameter('counts_per_rev', 4100)        # encoder CPR
        self.declare_parameter('turning_gain', 1.0)           # keep 1.0; tune if needed
        self.declare_parameter('publish_hz', 20.0)            # 20 Hz default
        self.declare_parameter('smoothing_alpha', 0.3)        # EMA alpha for v & omega
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        # Covariances (variance values)
        self.declare_parameter('pose_cov_xy', 0.02)           # m^2
        self.declare_parameter('pose_cov_yaw', 0.05)          # rad^2
        self.declare_parameter('twist_cov_v', 0.02)           # (m/s)^2
        self.declare_parameter('twist_cov_omega', 0.02)      # (rad/s)^2

        # load params
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_separation = float(self.get_parameter('wheel_separation').value)
        self.counts_per_rev = float(self.get_parameter('counts_per_rev').value)
        self.turning_gain = float(self.get_parameter('turning_gain').value)
        self.publish_hz = float(self.get_parameter('publish_hz').value)
        self.alpha = float(self.get_parameter('smoothing_alpha').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.pose_cov_xy = float(self.get_parameter('pose_cov_xy').value)
        self.pose_cov_yaw = float(self.get_parameter('pose_cov_yaw').value)
        self.twist_cov_v = float(self.get_parameter('twist_cov_v').value)
        self.twist_cov_omega = float(self.get_parameter('twist_cov_omega').value)

        # --- Internal state ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.v = 0.0                 # filtered linear speed
        self.omega = 0.0             # filtered angular speed

        self.last_counts = None      # store previous encoder counts as list [rl, rr]
        self.last_time = None        # rclpy.time.Time

        # Derived
        self.ticks_to_m = (2.0 * math.pi * self.wheel_radius) / self.counts_per_rev

        # Publisher / Broadcaster / Subscriber
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Int32MultiArray, 'encoder_counts', self.encoder_callback, 10)

        # Log file for debugging (optional)
        # CSV log files
        self.encoder_log_file = os.path.expanduser('~/encoder_counts.csv')
        self.delta_log_file   = os.path.expanduser('~/deltas.csv')
        self.vel_log_file     = os.path.expanduser('~/velocities.csv')

        # Initialize CSV files with headers
        for file, header in [(self.encoder_log_file, 'timestamp,RL,RR'),
                            (self.delta_log_file, 'timestamp,delta_RL,delta_RR,dt'),
                            (self.vel_log_file, 'timestamp,raw_v,raw_omega,filtered_v,filtered_omega')]:
            try:
                with open(file, 'w') as f:
                    f.write(header + '\n')
            except Exception:
                pass


        # Timer for publishing at given rate
        period = 1.0 / max(1.0, self.publish_hz)
        self.create_timer(period, self.publish_odom)

        self.get_logger().info(f"OdomPublisher ready: {self.odom_frame} -> {self.base_frame} @ {self.publish_hz} Hz")

    def encoder_callback(self, msg: Int32MultiArray):
        counts = msg.data
        if len(counts) < 2:
            self.get_logger().warn("encoder_counts must be [RL, RR]")
            return

        now = self.get_clock().now()

        # initialize on first real message
        if self.last_counts is None:
            self.last_counts = [int(counts[0]), int(counts[1])]
            self.last_time = now
            return

        # compute dt (seconds)
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            # skip invalid dt
            return

        # compute deltas in ticks
        delta_RL = int(counts[0]) - int(self.last_counts[0])
        delta_RR = int(counts[1]) - int(self.last_counts[1])

        # convert to meters
        dist_RL = delta_RL * self.ticks_to_m
        dist_RR = delta_RR * self.ticks_to_m

        # distances left/right (assume RL is left, RR is right)
        left_dist = dist_RL
        right_dist = dist_RR

        # linear velocity (m/s) and raw angular (rad/s)
        raw_v = (left_dist + right_dist) / (2.0 * dt)
        raw_omega = (right_dist - left_dist) / (self.wheel_separation * dt)  # rad/s

        # apply turning_gain (kept minimal, default 1.0) and smoothing (EMA)
        omega_scaled = raw_omega * self.turning_gain
        self.v = (self.alpha * raw_v) + ((1.0 - self.alpha) * self.v)
        self.omega = (self.alpha * omega_scaled) + ((1.0 - self.alpha) * self.omega)

        # integrate pose (simple Euler integration)
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta += self.omega * dt

        # normalize theta to [-pi, pi]
        self.theta = (self.theta + math.pi) % (2.0 * math.pi) - math.pi

        # save for next callback
        # copy values to avoid aliasing with incoming msg.data list
        self.last_counts = [int(counts[0]), int(counts[1])]
        self.last_time = now

        # debug log line: dt, dRL, dRR, raw_v, raw_omega, v_filtered, omega_filtered
        # log raw encoder counts
        try:
            with open(self.encoder_log_file, 'a') as f:
                f.write(f"{now.nanoseconds},{counts[0]},{counts[1]}\n")
        except Exception:
            pass

        # log deltas
        try:
            with open(self.delta_log_file, 'a') as f:
                f.write(f"{now.nanoseconds},{delta_RL},{delta_RR},{dt:.6f}\n")
        except Exception:
            pass

        # log velocities
        try:
            with open(self.vel_log_file, 'a') as f:
                f.write(f"{now.nanoseconds},{raw_v:.6f},{raw_omega:.6f},{self.v:.6f},{self.omega:.6f}\n")
        except Exception:
            pass

    def publish_odom(self):
        now = self.get_clock().now()

        # prepare tf
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
        # send transform
        self.tf_broadcaster.sendTransform(t)

        # prepare odom message
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

        # fill pose covariance (36 floats: row-major)
        # We set reasonable diagonal values: x,y variance and yaw variance,
        # other covariances left as zero.
        pose_cov = [0.0] * 36
        pose_cov[0] = self.pose_cov_xy  # x variance
        pose_cov[7] = self.pose_cov_xy  # y variance (index 7: row1 col1 etc)
        pose_cov[35] = self.pose_cov_yaw # yaw variance (index 35)
        odom.pose.covariance = pose_cov

        # twist
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.omega

        # twist covariance (36 floats)
        twist_cov = [0.0] * 36
        twist_cov[0] = self.twist_cov_v
        twist_cov[35] = self.twist_cov_omega
        odom.twist.covariance = twist_cov

        # publish
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
