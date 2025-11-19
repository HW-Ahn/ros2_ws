# 파일: tb3_security_system/nodes/simple_navigator.py

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


def quat_to_yaw(x, y, z, w):
    """쿼터니언 -> yaw(라디안)"""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class SimpleNavigator(Node):
    def __init__(self):
        super().__init__('simple_navigator')

        # 파라미터
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.8)
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('obstacle_distance', 0.4)
        self.declare_parameter('obstacle_angle_deg', 30.0)

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.obstacle_distance = self.get_parameter('obstacle_distance').value
        self.obstacle_angle = math.radians(
            self.get_parameter('obstacle_angle_deg').value
        )

        # 상태 변수
        self.has_goal = False
        self.goal_x = 0.0
        self.goal_y = 0.0

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.have_odom = False

        self.last_scan = None

        # 통신 설정
        self.cmd_goal_sub = self.create_subscription(
            PoseStamped, 'cmd_goal', self.goal_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10
        )

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.goal_reached_pub = self.create_publisher(Bool, 'goal_reached', 10)

        # 주기적 제어 루프 (20Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('SimpleNavigator started')

    def goal_callback(self, msg: PoseStamped):
        # frame_id는 'odom'이라고 가정
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.has_goal = True
        self.get_logger().info(
            f'New goal received: x={self.goal_x:.2f}, y={self.goal_y:.2f}'
        )

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        self.have_odom = True

    def scan_callback(self, msg: LaserScan):
        self.last_scan = msg

    def publish_stop(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def check_obstacle(self):
        """전방 근처 장애물 있는지 확인, 좌/우 거리도 같이 계산."""
        if self.last_scan is None:
            return False, 0.0, 0.0

        scan = self.last_scan
        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        ranges = scan.ranges

        front_min = float('inf')
        left_min = float('inf')
        right_min = float('inf')

        for i, r in enumerate(ranges):
            angle = angle_min + i * angle_inc
            if math.isinf(r) or math.isnan(r):
                continue

            # 전방
            if abs(angle) < self.obstacle_angle:
                front_min = min(front_min, r)
            # 좌측
            if 0.0 < angle < math.pi / 2.0:
                left_min = min(left_min, r)
            # 우측
            if -math.pi / 2.0 < angle < 0.0:
                right_min = min(right_min, r)

        obstacle = front_min < self.obstacle_distance
        return obstacle, left_min, right_min

    def control_loop(self):
        if not self.has_goal or not self.have_odom:
            return

        # 목표까지 거리 계산
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        dist = math.hypot(dx, dy)

        if dist < self.goal_tolerance:
            # 목표 도착
            self.publish_stop()
            self.has_goal = False
            msg = Bool()
            msg.data = True
            self.goal_reached_pub.publish(msg)
            self.get_logger().info('Goal reached.')
            return

        twist = Twist()

        # 장애물 확인
        obstacle, left_min, right_min = self.check_obstacle()
        if obstacle:
            # 전방에 장애물 → 회피 회전
            self.get_logger().debug(
                f'Obstacle detected. left_min={left_min:.2f}, right_min={right_min:.2f}'
            )
            twist.linear.x = 0.0
            # 더 넓은 쪽으로 회전
            if left_min > right_min:
                twist.angular.z = self.angular_speed
            else:
                twist.angular.z = -self.angular_speed
            self.cmd_vel_pub.publish(twist)
            return

        # 장애물 없으면 목표 방향으로 전진
        target_yaw = math.atan2(dy, dx)
        yaw_error = self.normalize_angle(target_yaw - self.current_yaw)

        # yaw 오차가 크면 회전 위주
        if abs(yaw_error) > math.radians(15.0):
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed if yaw_error > 0.0 else -self.angular_speed
        else:
            # 조금만 보정하면서 전진
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.5 * yaw_error

        self.cmd_vel_pub.publish(twist)

    @staticmethod
    def normalize_angle(a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a


def main(args=None):
    rclpy.init(args=args)
    node = SimpleNavigator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
