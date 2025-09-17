#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import PoseArray

class IgnPoseToTF(Node):
    def __init__(self):
        super().__init__('ign_pose_to_tf')
        self.tf_broadcaster = TransformBroadcaster(self)
        # Subscribe to the PoseArray coming from Ignition
        self.subscription = self.create_subscription(
            PoseArray,
            '/model/dekut_amr/tf',
            self.pose_array_callback,
            10
        )
        self.get_logger().info('Subscribed to /model/dekut_amr/tf')

    def pose_array_callback(self, msg: PoseArray):
        # For each pose in PoseArray, create a TF
        for i, pose in enumerate(msg.poses):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'odom' if i == 0 else 'base_link'  # parent frame
            t.child_frame_id = 'base_link' if i == 0 else f'sensor_link_{i}'
            t.transform.translation.x = pose.position.x
            t.transform.translation.y = pose.position.y
            t.transform.translation.z = pose.position.z
            t.transform.rotation.x = pose.orientation.x
            t.transform.rotation.y = pose.orientation.y
            t.transform.rotation.z = pose.orientation.z
            t.transform.rotation.w = pose.orientation.w
            self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = IgnPoseToTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
