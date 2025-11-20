# 파일: ros2_ws/src/tb3_security_system/tb3_security_system/nodes/obstacle_monitor.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class ObstacleMonitor(Node):
    def __init__(self):
        super().__init__('obstacle_monitor')
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_cb, 10)
        self.alarm_pub = self.create_publisher(Bool, 'alarm_event', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.prev_min = None
        self.get_logger().info('ObstacleMonitor ready')

    def scan_cb(self, msg: LaserScan):
        # 간단한 히스토리: 현재 최소거리와 이전 최소거리 차이로 "움직이는 장애물" 판단
        # 동작 조건: 최소거리 < 0.6m 이고 이전 값 - 현재 값 > 0.2 (갑작스럽게 다가옴) => moving
        valid_ranges = [r for r in msg.ranges if r > msg.range_min and r < msg.range_max]
        if not valid_ranges:
            return
        cur_min = min(valid_ranges)
        if self.prev_min is not None:
            if cur_min < 0.6 and (self.prev_min - cur_min) > 0.15:
                # moving obstacle detected
                self.get_logger().warn('Moving obstacle detected -> publish alarm_event True')
                self.alarm_pub.publish(Bool(data=True))
                # additionally, publish a small rotation command for immediate feedback (optional)
                twist = Twist()
                twist.angular.z = -1.0  # 시계 방향
                self.cmd_vel_pub.publish(twist)
                return
        # no moving obstacle
        self.prev_min = cur_min

def main(args=None):
  rclpy.init(args=args)
  node = ObstacleMonitor()  # 클래스 이름에 맞게 수정
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()

