# tb3_security_system/nodes/qr_scanner.py

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class QRScanner(Node):
    def __init__(self):
        super().__init__('qr_scanner')

        self.declare_parameter("camera_topic", "/tb3_1/image_raw/compressed")
        cam_topic = self.get_parameter("camera_topic").value

        self.bridge = CvBridge()
        self.detector = cv2.QRCodeDetector()

        self.sub = self.create_subscription(
            CompressedImage,
            cam_topic,
            self.image_cb,
            10
        )

        self.pub = self.create_publisher(String, "/tb3_1/qr_result", 10)

        self.get_logger().info(f"[QRScanner] Subscribing camera: {cam_topic}")

    def image_cb(self, msg):
        np_img = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_img, cv2.IMREAD_COLOR)

        if frame is None:
            return

        data, bbox, _ = self.detector.detectAndDecode(frame)

        if data:
            result = String()
            result.data = data
            self.pub.publish(result)
            self.get_logger().info(f"[QRScanner] QR Detected: {data}")

def main():
    rclpy.init()
    node = QRScanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
