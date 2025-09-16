#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')

        self.declare_parameter('roi_size', 100)
        self.declare_parameter('color_threshold', 500)
        self.declare_parameter('show_image', True)

        self.roi_size = self.get_parameter('roi_size').value
        self.color_threshold = self.get_parameter('color_threshold').value
        self.show_image = self.get_parameter('show_image').value

        self.subscription = self.create_subscription(
            CompressedImage,
            'camera/image/compressed',
            self.listener_callback,
            20
        )

        self.color_ranges = {
            "red1":    ([0, 120, 70], [10, 255, 255]),
            "red2":    ([170, 120, 70], [180, 255, 255]),
            "green":   ([40, 50, 50], [90, 255, 255]),
            "blue":    ([100, 150, 0], [140, 255, 255]),
            "yellow":  ([20, 100, 100], [30, 255, 255]),
            "orange":  ([10, 100, 20], [25, 255, 255]),
        }

    def listener_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            self.get_logger().warn("Failed to decode image")
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, w, _ = hsv.shape

        # Region of interest (center square)
        half = self.roi_size // 2
        roi = hsv[h//2-half:h//2+half, w//2-half:w//2+half]

        detected_color = "None"
        for color, (lower, upper) in self.color_ranges.items():
            lower = np.array(lower)
            upper = np.array(upper)
            mask = cv2.inRange(roi, lower, upper)
            if cv2.countNonZero(mask) > self.color_threshold:
                detected_color = "red" if "red" in color else color
                break

        self.get_logger().info(f"Detected Color: {detected_color}")

        if self.show_image:
            cv2.rectangle(frame, (w//2-half, h//2-half), (w//2+half, h//2+half), (255,255,255), 2)
            cv2.imshow("Camera Subscriber", frame)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down subscriber...")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
