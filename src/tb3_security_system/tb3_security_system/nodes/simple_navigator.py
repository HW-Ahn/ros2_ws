# 파일: tb3_security_system/nodes/simple_navigator.py

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


def quat_to_yaw(x, y, z, w):
    """쿼터니언 -> yaw(라디안)"""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class SimpleNavigator(Node):
    def __init__(self):
        super().__init__('simple_navigator')

        # 기본 이동/허용 오차/장애물 파라미터
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.8)
        self.declare_parameter('pos_tolerance', 0.15)
        self.declare_parameter('yaw_tolerance_deg', 10.0)
        self.declare_parameter('front_angle_deg', 15.0)
        self.declare_parameter('obstacle_distance', 0.40)
        self.declare_parameter('critical_distance', 0.25)

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.pos_tolerance = float(self.get_parameter('pos_tolerance').value)
        self.yaw_tolerance = math.radians(
            float(self.get_parameter('yaw_tolerance_deg').value)
        )
        self.front_angle = math.radians(
            float(self.get_parameter('front_angle_deg').value)
        )
        self.obstacle_distance = float(self.get_parameter('obstacle_distance').value)
        self.critical_distance = float(self.get_parameter('critical_distance').value)

        # 상태 변수
        self.has_goal = False
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_yaw = 0.0

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.have_odom = False

        self.last_scan = None

        # 회피 상태: NONE / TURN / ARC / BACK_OFF
        self.avoid_state = 'NONE'
        self.avoid_clear_count = 0
        self.avoid_clear_needed = 5

        # 통신 설정
        self.cmd_goal_sub = self.create_subscription(
            PoseStamped, 'cmd_goal', self.goal_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10
        )
        scan_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=scan_qos
        )
        self.nav_cancel_sub = self.create_subscription(
            Bool, 'nav_cancel', self.nav_cancel_callback, 10
        )
        self.manual_sub = self.create_subscription(
            Twist, 'cmd_vel_manual', self.manual_cmd_callback, 10
        )

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.goal_reached_pub = self.create_publisher(Bool, 'goal_reached', 10)

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('SimpleNavigator started')

    # --------------------- 콜백들 ---------------------
    def goal_callback(self, msg: PoseStamped):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        q = msg.pose.orientation
        self.goal_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

        self.has_goal = True
        self.avoid_state = 'NONE'
        self.avoid_clear_count = 0

        self.get_logger().info(
            f'New goal: x={self.goal_x:.2f}, y={self.goal_y:.2f}, '
            f'yaw={math.degrees(self.goal_yaw):.1f}deg'
        )

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        self.have_odom = True

    def scan_callback(self, msg: LaserScan):
        self.last_scan = msg

    def nav_cancel_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info('Navigation canceled by external request')
            self.has_goal = False
            self.avoid_state = 'NONE'
            self.avoid_clear_count = 0
            self.publish_stop()

    def manual_cmd_callback(self, msg: Twist):
        # 목표가 없을 때만 수동 제어를 /cmd_vel로 전달
        if not self.has_goal:
            self.cmd_vel_pub.publish(msg)

    # --------------------- 유틸 ---------------------
    def publish_stop(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def check_front_obstacle(self):
        if self.last_scan is None:
            return False, float('inf')

        scan = self.last_scan
        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        ranges = scan.ranges

        # 정면 중심 각도 (일단 0rad로 가정, 필요하면 파라미터로 조정 가능)
        front_center = 0.0
        half = self.front_angle  # front_angle_deg 를 rad 로 바꿔둔 값

        min_front = float('inf')

        for i, r in enumerate(ranges):
            # 0.0, range_min 이하, range_max 이상, inf/nan 은 전부 무시
            if (
                r == 0.0
                or math.isinf(r)
                or math.isnan(r)
                or r < scan.range_min
                or r > scan.range_max
            ):
                continue

            angle = angle_min + i * angle_inc

            # 현재 각도를 정면 기준으로 정규화(-π ~ π)
            rel = self.normalize_angle(angle - front_center)

            if -half <= rel <= half:
                if r < min_front:
                    min_front = r

        return (min_front < self.obstacle_distance), min_front

    @staticmethod
    def normalize_angle(a: float) -> float:
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    # --------------------- 메인 제어 루프 ---------------------
    def control_loop(self):
        if not self.has_goal or not self.have_odom:
            return

        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        dist = math.hypot(dx, dy)

        pos_ok = dist < self.pos_tolerance
        yaw_error = self.normalize_angle(self.goal_yaw - self.current_yaw)
        yaw_ok = abs(yaw_error) < self.yaw_tolerance

        # 1) 도착
        if pos_ok and yaw_ok:
            self.publish_stop()
            self.has_goal = False
            self.avoid_state = 'NONE'
            self.avoid_clear_count = 0

            msg = Bool()
            msg.data = True
            self.goal_reached_pub.publish(msg)

            self.get_logger().info(
                f'Goal reached (dist={dist:.3f}, yaw_err={math.degrees(yaw_error):.1f}deg)'
            )
            return

        twist = Twist()

        # 2) 위치는 거의 맞는데 yaw만 남은 경우 → 제자리에서 정렬
        if pos_ok and not yaw_ok and self.avoid_state == 'NONE':
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed if yaw_error > 0.0 else -self.angular_speed
            self.cmd_vel_pub.publish(twist)
            return

        # 3) 정면 장애물 체크
        obstacle, front_min = self.check_front_obstacle()

        # 너무 가까우면 BACK_OFF
        if obstacle and front_min < self.critical_distance:
            self.avoid_state = 'BACK_OFF'
            self.avoid_clear_count = 0

        # 적당히 가까우면 TURN → ARC
        elif obstacle and self.avoid_state == 'NONE':
            self.avoid_state = 'TURN'
            self.avoid_clear_count = 0
            self.get_logger().info(f'Obstacle detected at {front_min:.2f} m, start TURN')

        # 장애물이 안 보이면 회피 종료 카운트
        if not obstacle and self.avoid_state in ['TURN', 'ARC']:
            self.avoid_clear_count += 1
            if self.avoid_clear_count >= self.avoid_clear_needed:
                self.get_logger().info('Obstacle cleared, stop avoidance')
                self.avoid_state = 'NONE'
                self.avoid_clear_count = 0

        # 4) 회피 상태별 동작
        if self.avoid_state == 'BACK_OFF':
            twist.linear.x = -0.05
            twist.angular.z = self.angular_speed
            self.cmd_vel_pub.publish(twist)

            if front_min > self.critical_distance * 1.5:
                self.avoid_state = 'TURN'
            return

        if self.avoid_state == 'TURN':
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed
            self.cmd_vel_pub.publish(twist)

            if not obstacle or front_min > self.obstacle_distance * 1.3:
                self.avoid_state = 'ARC'
                self.get_logger().info('Switch TURN -> ARC')
            return

        if self.avoid_state == 'ARC':
            twist.linear.x = 0.08
            twist.angular.z = self.angular_speed * 0.5
            self.cmd_vel_pub.publish(twist)
            return

        # 5) 회피 상태가 아니면 목표 방향으로 정상 이동
        target_yaw = math.atan2(dy, dx)
        heading_error = self.normalize_angle(target_yaw - self.current_yaw)

        if abs(heading_error) > math.radians(25.0):
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed if heading_error > 0.0 else -self.angular_speed
        else:
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.7 * heading_error

        self.cmd_vel_pub.publish(twist)


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
