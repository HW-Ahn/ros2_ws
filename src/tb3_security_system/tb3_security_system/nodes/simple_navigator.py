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

        # 정면 감지 각도 (기본 ±25도로 살짝 넓힘)
        self.declare_parameter('front_angle_deg', 25.0)

        # 라이다 기준 "앞" 방향 (deg, 필요시 튜닝용)
        self.declare_parameter('front_center_deg', 0.0)

        # 좌/우측 비교용 사이드 섹터 범위 (예: 정면 기준 ±80도까지)
        self.declare_parameter('side_angle_deg', 80.0)

        self.declare_parameter('obstacle_distance', 0.40)
        self.declare_parameter('critical_distance', 0.25)

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.pos_tolerance = float(self.get_parameter('pos_tolerance').value)
        self.yaw_tolerance = math.radians(
            float(self.get_parameter('yaw_tolerance_deg').value)
        )

        # 정면/좌우 섹터 설정
        self.front_angle = math.radians(
            float(self.get_parameter('front_angle_deg').value)
        )
        self.front_center = math.radians(
            float(self.get_parameter('front_center_deg').value)
        )
        self.side_angle = math.radians(
            float(self.get_parameter('side_angle_deg').value)
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

        # 회피 방향: +1.0(좌회전), -1.0(우회전)
        self.avoid_turn_dir = 1.0

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
        # 처음 한 번만 라이다 정보 로그
        if not hasattr(self, 'printed_scan_info'):
            self.printed_scan_info = True
            self.get_logger().info(
                f'scan angle_min={msg.angle_min:.3f}, angle_max={msg.angle_max:.3f}, '
                f'angle_inc={msg.angle_increment:.4f}, len={len(msg.ranges)}'
            )

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

    @staticmethod
    def normalize_angle(a: float) -> float:
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def analyze_obstacles(self):
        """
        라이다 데이터에서 정면/좌/우 섹터의 최소 거리 계산.
        - 정면: front_center ± front_angle
        - 좌측: front_center + front_angle ~ +side_angle
        - 우측: front_center - side_angle ~ -front_angle
        """
        if self.last_scan is None:
            return False, float('inf'), float('inf'), float('inf')

        scan = self.last_scan
        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        ranges = scan.ranges

        min_front = float('inf')
        min_left = float('inf')
        min_right = float('inf')

        for i, r in enumerate(ranges):
            # 0.0, 유효 범위 밖, inf/nan 값은 모두 무시
            if (
                r == 0.0
                or math.isinf(r)
                or math.isnan(r)
                or r < scan.range_min
                or r > scan.range_max
            ):
                continue

            angle = angle_min + i * angle_inc
            # 라이다 기준 "앞" 방향(self.front_center)을 0으로 맞춰서 정규화
            rel = self.normalize_angle(angle - self.front_center)

            # 정면 섹터
            if -self.front_angle <= rel <= self.front_angle:
                if r < min_front:
                    min_front = r

            # 좌측 섹터 (앞보다 왼쪽)
            elif self.front_angle < rel <= self.side_angle:
                if r < min_left:
                    min_left = r

            # 우측 섹터 (앞보다 오른쪽)
            elif -self.side_angle <= rel < -self.front_angle:
                if r < min_right:
                    min_right = r

        obstacle_front = (min_front < self.obstacle_distance)
        return obstacle_front, min_front, min_left, min_right

    def choose_avoid_direction(self, min_left: float, min_right: float) -> float:
        """
        좌/우측 최소 거리를 비교해서 더 여유 있는 방향으로 회피.
        return: +1.0 (좌회전), -1.0 (우회전)
        """
        # inf(측정 없음)는 충분히 멀리 떨어진 것으로 간주
        def safe_dist(d):
            return d if d < float('inf') else self.obstacle_distance * 3.0

        safe_left = safe_dist(min_left)
        safe_right = safe_dist(min_right)

        # 약간의 여유(0.05m) 차이가 날 때만 방향을 바꿔줌
        if safe_left > safe_right + 0.05:
            return +1.0   # 왼쪽이 더 여유 → 좌회전
        elif safe_right > safe_left + 0.05:
            return -1.0   # 오른쪽이 더 여유 → 우회전
        else:
            # 거의 비슷하면 기본 좌회전
            return +1.0

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

        # 목표 방향 계산
        target_yaw = math.atan2(dy, dx)
        heading_error = self.normalize_angle(target_yaw - self.current_yaw)

        # 기본 목표 추종 명령 (회피 상태가 아닐 때 사용)
        if abs(heading_error) > math.radians(25.0):
            # 아직 방향을 많이 틀어야 함 → 제자리 회전
            base_linear = 0.0
            base_angular = self.angular_speed if heading_error > 0.0 else -self.angular_speed
        else:
            # 대략 목표 쪽을 보고 있음 → 전진 + 약간의 회전
            base_linear = self.linear_speed
            base_angular = 0.7 * heading_error

        # 2) 위치는 거의 맞는데 yaw만 남은 경우 → 제자리에서 정렬 (회피 OFF)
        if pos_ok and not yaw_ok and self.avoid_state == 'NONE':
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed if yaw_error > 0.0 else -self.angular_speed
            self.cmd_vel_pub.publish(twist)
            return

        # 회피/장애물 정보 초기값
        obstacle_front = False
        front_min = float('inf')
        min_left = float('inf')
        min_right = float('inf')

        # 회피 중이거나, 이번 루프에서 전진하려고 할 때만 라이다 체크
        need_scan = (self.avoid_state != 'NONE') or (base_linear > 0.0)

        if need_scan:
            obstacle_front, front_min, min_left, min_right = self.analyze_obstacles()

        # 장애물이 안 보이면 TURN/ARC 클리어 카운트 증가
        if not obstacle_front and self.avoid_state in ['TURN', 'ARC']:
            self.avoid_clear_count += 1
            if self.avoid_clear_count >= self.avoid_clear_needed:
                self.get_logger().info('Obstacle cleared, stop avoidance')
                self.avoid_state = 'NONE'
                self.avoid_clear_count = 0

        # 3) 전진하려는 상황에서만 새로 회피 모드 진입
        if base_linear > 0.0:
            # 너무 가까우면 BACK_OFF
            if obstacle_front and front_min < self.critical_distance:
                self.avoid_state = 'BACK_OFF'
                self.avoid_clear_count = 0
                self.avoid_turn_dir = self.choose_avoid_direction(min_left, min_right)
                self.get_logger().info(
                    f'Front obstacle VERY close ({front_min:.2f} m), BACK_OFF, dir={self.avoid_turn_dir}'
                )

            # 적당히 가까우면 TURN → ARC
            elif obstacle_front and self.avoid_state == 'NONE':
                self.avoid_state = 'TURN'
                self.avoid_clear_count = 0
                self.avoid_turn_dir = self.choose_avoid_direction(min_left, min_right)
                self.get_logger().info(
                    f'Front obstacle at {front_min:.2f} m, start TURN, dir={self.avoid_turn_dir}'
                )

        # 4) 회피 상태별 동작
        if self.avoid_state == 'BACK_OFF':
            twist.linear.x = -0.05
            twist.angular.z = self.angular_speed * self.avoid_turn_dir
            self.cmd_vel_pub.publish(twist)

            # 일정 거리 이상 떨어지면 TURN 으로 전환
            if not obstacle_front or front_min > self.critical_distance * 1.5:
                self.avoid_state = 'TURN'
                self.avoid_clear_count = 0
                self.get_logger().info('BACK_OFF -> TURN')
            return

        if self.avoid_state == 'TURN':
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed * self.avoid_turn_dir
            self.cmd_vel_pub.publish(twist)

            # 정면이 어느 정도 비면 ARC 로 전환
            if not obstacle_front or front_min > self.obstacle_distance * 1.3:
                self.avoid_state = 'ARC'
                self.avoid_clear_count = 0
                self.get_logger().info('TURN -> ARC')
            return

        if self.avoid_state == 'ARC':
            twist.linear.x = 0.08
            twist.angular.z = self.angular_speed * 0.5 * self.avoid_turn_dir
            self.cmd_vel_pub.publish(twist)

            # ARC 중에도 너무 가까워지면 BACK_OFF 재진입
            if obstacle_front and front_min < self.critical_distance:
                self.avoid_state = 'BACK_OFF'
                self.avoid_clear_count = 0
                self.get_logger().info('ARC -> BACK_OFF (too close)')
            return

        # 5) 회피 상태가 아니면 목표 방향으로 정상 이동
        twist.linear.x = base_linear
        twist.angular.z = base_angular
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
