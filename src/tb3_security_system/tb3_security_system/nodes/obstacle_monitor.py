# 파일: tb3_security_system/nodes/obstacle_monitor.py

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class ObstacleMonitor(Node):
    def __init__(self):
        super().__init__('obstacle_monitor')

        # 네임스페이스 기반으로 동작 (tb3_1, tb3_2 모두 재사용할 수 있게)
        ns = self.get_namespace().strip('/')
        self.robot_ns = ns if ns else 'tb3_1'

        # 전방 장애물 거리 임계값 (m)
        self.front_threshold = 0.5

        # 얼마나 넓은 각도를 "정면"으로 볼지 (rad)
        self.front_angle = math.radians(60.0)  # ±30도

        # alarm_event 퍼블리셔 (BehaviorManager 가 구독)
        self.alarm_pub = self.create_publisher(Bool, 'alarm_event', 10)

        # LaserScan 구독
        # 실제 토픽 이름은 ros2 topic list | grep scan 으로 확인해서 맞춰주세요.
        # 예: /tb3_1/scan 또는 /scan
        scan_topic = 'scan'   # 네임스페이스 안에서 상대 경로로 사용
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self.scan_cb, 10
        )

        self.get_logger().info(
            f'ObstacleMonitor started for {self.robot_ns}, scan_topic={scan_topic}'
        )

    def scan_cb(self, msg: LaserScan):
        # LaserScan 의 각도 범위: [angle_min, angle_max], 증분 angle_increment
        angle = msg.angle_min
        min_dist_front = float('inf')

        for r in msg.ranges:
            # 유효하지 않은 값은 건너뛰기
            if math.isnan(r) or math.isinf(r):
                angle += msg.angle_increment
                continue

            # 정면 ±front_angle 안에 있는 빔만 확인
            if abs(angle) <= self.front_angle:
                if r < min_dist_front:
                    min_dist_front = r

            angle += msg.angle_increment

        # 장애물 여부 판단
        alarm = Bool()
        if min_dist_front < self.front_threshold:
            alarm.data = True
            self.get_logger().warn(
                f'Front obstacle detected: {min_dist_front:.2f} m'
            )
        else:
            alarm.data = False

        self.alarm_pub.publish(alarm)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
