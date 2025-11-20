# ros_process.py (디버깅 코드 추가 버전)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import time
from multiprocessing import Queue

# -----------------------------
# ROS 통신 담당 노드
# -----------------------------
class CompressedImageSubscriber(Node):
    def __init__(self, queue, topic_name):
        # rclpy 노드 이름은 유니크해야 합니다.
        self.node_name = f'camera_process_node_{topic_name.replace("/", "_")}'
        super().__init__(self.node_name)

        self.queue = queue
        self.topic_name = topic_name
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            CompressedImage,
            topic_name,
            self.image_callback,
            10  # QoS profile depth
        )
        self.get_logger().info(f'ROS Subscriber setup for topic: {topic_name}')

    def image_callback(self, msg):
        """압축된 이미지 메시지를 받아서 OpenCV Mat 형태로 변환하고 큐에 넣습니다."""
        try:
            # 압축 이미지 메시지를 OpenCV Mat 형태로 변환
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

            # 🛑 디버깅 코드: ROS 데이터 수신 및 큐 전송 확인
            print(f"[{self.node_name}] INFO: Image Received & Sending to Queue ({cv_image.shape[:2]})")

            # 큐가 가득 찼다면 (maxsize=1이므로) 오래된 데이터를 버립니다.
            if not self.queue.empty():
                try:
                    self.queue.get_nowait()
                except:
                    pass

            # 새로운 데이터를 큐에 넣습니다.
            self.queue.put((cv_image, self.topic_name), block=False)

        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

def ros_spin_process(queue: Queue, topic_name: str):
    """별도의 프로세스에서 ROS 노드를 초기화하고 스핀합니다."""
    # ROS 2가 이미 초기화되었는지 확인하고, 아니면 초기화합니다.
    if not rclpy.ok():
        rclpy.init()

    node = CompressedImageSubscriber(queue, topic_name)

    try:
        # 노드를 블로킹 방식으로 실행합니다.
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 노드 및 rclpy 종료
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    # 이 파일이 단독으로 실행되는 경우를 대비한 보호 코드
    print("This file should be imported, not run directly.")
