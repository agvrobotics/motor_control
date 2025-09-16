#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            'camera/image/compressed',
            self.process_frame,
            20
        )

        # HSV color ranges for detection (focus on red and blue)
        self.color_ranges = {
            "red":  ([0, 120, 70], [10, 255, 255]),
            "blue": ([100, 150, 0], [140, 255, 255])
        }

    def process_frame(self, msg):
        # Convert ROS message to OpenCV image
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn("Failed to decode image")
            return

        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Use center region (100x100 px)
        h, w, _ = hsv.shape
        roi = hsv[h//2-50:h//2+50, w//2-50:w//2+50]

        detected_color = "None"
        for color, (lower, upper) in self.color_ranges.items():
            mask = cv2.inRange(roi, np.array(lower), np.array(upper))
            if cv2.countNonZero(mask) > 500:  # adjust if needed
                detected_color = color
                break

        print("Detected Color:", detected_color)

        # Draw ROI for visualization
        cv2.rectangle(frame, (w//2-50, h//2-50), (w//2+50, h//2+50), (255, 255, 255), 2)
        cv2.imshow("Camera Subscriber", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
