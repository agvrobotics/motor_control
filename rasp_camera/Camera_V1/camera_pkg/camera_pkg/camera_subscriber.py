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
            self.listener_callback,
            20  # queue size
        )
        self.subscription  # prevent unused variable warning

        # Define HSV color ranges
        self.color_ranges = {
            "red":    ([0, 120, 70], [10, 255, 255]),
            "green":  ([40, 50, 50], [90, 255, 255]),
            "blue":   ([100, 150, 0], [140, 255, 255]),
            "yellow": ([20, 100, 100], [30, 255, 255]),
            "orange": ([10, 100, 20], [25, 255, 255]),
        }

    def listener_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is not None:
            # Convert to HSV
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Optionally, use a region of interest (center 100x100 px)
            h, w, _ = hsv.shape
            roi = hsv[h//2-50:h//2+50, w//2-50:w//2+50]

            detected_color = "None"
            for color, (lower, upper) in self.color_ranges.items():
                lower = np.array(lower)
                upper = np.array(upper)
                mask = cv2.inRange(roi, lower, upper)
                if cv2.countNonZero(mask) > 500:  # adjust threshold
                    detected_color = color
                    break

            print("Detected Color:", detected_color)

            # Optional: display frame
            cv2.rectangle(frame, (w//2-50, h//2-50), (w//2+50, h//2+50), (255,255,255), 2)
            cv2.imshow("Camera Subscriber", frame)
            cv2.waitKey(1)
        else:
            self.get_logger().warn("Failed to decode image")

def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
