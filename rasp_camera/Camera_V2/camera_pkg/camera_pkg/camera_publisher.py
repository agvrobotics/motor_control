#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        self.declare_parameter('camera_index', 0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('jpeg_quality', 80)  # 0–100

        index = self.get_parameter('camera_index').value
        w = self.get_parameter('frame_width').value
        h = self.get_parameter('frame_height').value
        fps = self.get_parameter('fps').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value

        self.publisher_ = self.create_publisher(CompressedImage, 'camera/image/compressed', 20)

        # Open camera
        self.cap = cv2.VideoCapture(index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        if not self.cap.isOpened():
            self.get_logger().error(f'Could not open camera at index {index}')

        self.timer = self.create_timer(1.0 / fps, self.timer_callback)

        self.warned = False

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            if not self.warned:
                self.get_logger().warn('Failed to capture image')
                self.warned = True
            return
        self.warned = False

        _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = np.array(buffer).tobytes()

        self.publisher_.publish(msg)

    def cleanup(self):
        if self.cap.isOpened():
            self.cap.release()
            self.get_logger().info("Camera released successfully")


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down camera publisher...")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()
