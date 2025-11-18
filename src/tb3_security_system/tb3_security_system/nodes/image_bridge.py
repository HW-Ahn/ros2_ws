# 파일: tb3_security_system/nodes/image_bridge.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageBridge(Node):
    """
    tb3_1, tb3_2 카메라 이미지를 Python GUI로 전달하기 위한 브리지 노드.
    OpenCV 이미지로 변환하여 Qt 이벤트로 GUI로 보냄.
    """
    def __init__(self):
        super().__init__('image_bridge')
        self.bridge = CvBridge()

        # 두 로봇 카메라 구독
        self.sub_tb3_1 = self.create_subscription(
            Image,
            '/tb3_1/camera/image_raw',
            self.cb_tb3_1,
            10
        )
        self.sub_tb3_2 = self.create_subscription(
            Image,
            '/tb3_2/camera/image_raw',
            self.cb_tb3_2,
            10
        )

        # GUI가 얻게 될 전역 콜백 함수 저장 공간
        self.gui_update_cb_1 = None
        self.gui_update_cb_2 = None

    def register_gui_callbacks(self, cb1, cb2):
        """GUI(Qt)에서 호출하여 QLabel 업데이트 콜백 등록"""
        self.gui_update_cb_1 = cb1
        self.gui_update_cb_2 = cb2

    def cb_tb3_1(self, msg):
        if self.gui_update_cb_1 is None:
            return
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.gui_update_cb_1(cv_img)

    def cb_tb3_2(self, msg):
        if self.gui_update_cb_2 is None:
            return
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.gui_update_cb_2(cv_img)


def main():
    rclpy.init()
    node = ImageBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
