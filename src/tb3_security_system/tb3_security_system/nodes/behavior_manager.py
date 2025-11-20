# 파일: tb3_security_system/nodes/behavior_manager.py

import math
import random

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool, Int32
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry


def yaw_to_quat(yaw: float):
    """2D yaw(라디안) -> quaternion (x, y, z, w). 롤/피치는 0 가정."""
    half = yaw * 0.5
    cz = math.cos(half)
    sz = math.sin(half)
    # roll = pitch = 0
    return (0.0, 0.0, sz, cz)


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """quaternion -> 2D yaw(라디안)."""
    # 표준 변환식
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class BehaviorManager(Node):
    def __init__(self):
        super().__init__('behavior_manager')

        # 네임스페이스 확인 (예: /tb3_1, /tb3_2)
        ns = self.get_namespace().strip('/')
        self.robot_ns = ns if ns else 'tb3_1'
        self.is_primary = (self.robot_ns == 'tb3_1')  # tb3_1을 주 로봇으로 사용

        # -------------------------
        # 상태 변수
        # -------------------------
        self.current_mode = 'IDLE'
        self.current_qr = 1
        self.last_scanned_qr = 0

        # home_pose: 최초 odom 한 번만 저장 (자동 홈 위치)
        self.home_pose: PoseStamped | None = None

        # -------------------------
        # QR 스캔 위치 (이미 offset 반영된 좌표)
        # frame_id 는 모두 map 기준이라고 가정
        # -------------------------
        # 1)
        #  1번: x = -1.5, y = -2.3
        #  2번: x =  4.7, y = -2.3
        #  3번: x =  4.7, y =  2.1
        #  4번: x = -1.5, y = -2.1
        # yaw 은 기존 논리 유지:
        #   아래쪽 줄(1,2)은 "위(북쪽)를 바라보는" 방향 → -pi/2
        #   위쪽 줄(3,4)은 "아래(남쪽)를 바라보는" 방향 → +pi/2
        self.qr_scan_poses = {
            1: (-1.5, -2.3, -math.pi / 2.0),
            2: ( 4.7, -2.3, -math.pi / 2.0),
            3: ( 4.7,  2.1,  math.pi / 2.0),
            4: (-1.5, -2.1,  math.pi / 2.0),
        }

        # -------------------------
        # 퍼블리셔
        # -------------------------
        # GUI가 구독하는 상태 토픽: /tb3_X/state
        self.state_pub = self.create_publisher(String, 'state', 10)

        # simple_navigator 가 구독하는 goal 토픽: /tb3_X/cmd_goal
        self.goal_pub = self.create_publisher(PoseStamped, 'cmd_goal', 10)

        # simple_navigator 가 구독하는 수동 속도: /tb3_X/cmd_vel_manual
        # (기존 'cmd_vel' 이 아니라 'cmd_vel_manual' 로 맞춤)
        self.manual_vel_pub = self.create_publisher(Twist, 'cmd_vel_manual', 10)

        # 내비게이션 취소 (simple_navigator 에서 구독)
        self.nav_cancel_pub = self.create_publisher(Bool, 'nav_cancel', 10)

        # 교대 정보 공유 토픽 (멀티 로봇용, 나중을 위해 남겨둠)
        self.handover_pub = self.create_publisher(Int32, '/patrol/handover', 10)

        # -------------------------
        # 서브스크립션
        # -------------------------
        # GUI → high_level_cmd (GOTO_QR1, SEQ_PATROL_START, RANDOM_PATROL_START, RETURN_HOME, BUZZER, EMERGENCY_STOP 등)
        self.cmd_sub = self.create_subscription(
            String, 'high_level_cmd', self.cmd_cb, 10
        )

        # simple_navigator → goal_reached (목표점 도달 여부)
        self.goal_reached_sub = self.create_subscription(
            Bool, 'goal_reached', self.goal_reached_cb, 10
        )

        # 장애물 감지 (obstacle_monitor 등에서 publish 할 수 있음)
        self.alarm_sub = self.create_subscription(
            Bool, 'alarm_event', self.alarm_cb, 10
        )

        # 교대 정보 수신 (멀티 로봇용)
        self.handover_sub = self.create_subscription(
            Int32, '/patrol/handover', self.handover_cb, 10
        )

        # odom 구독: 최초 한 번만 home_pose 저장용
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_cb, 10
        )

        # 초기 상태
        self.set_mode('IDLE')
        self.get_logger().info(
            f'BehaviorManager started for {self.robot_ns}, primary={self.is_primary}'
        )

    # ============================================================
    # odom 콜백: home_pose 자동 저장
    # ============================================================
    def odom_cb(self, msg: Odometry):
        if self.home_pose is not None:
            return  # 이미 저장했으면 무시

        # 최초 한 번만 home_pose 설정
        self.home_pose = PoseStamped()
        # simple_navigator와 좌표계를 맞춘다고 생각하고, 여기서는 map을 기준이라고 가정
        self.home_pose.header.frame_id = 'map'
        self.home_pose.pose = msg.pose.pose

        yaw = quat_to_yaw(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        self.get_logger().info(
            f'[HOME] home_pose set: x={self.home_pose.pose.position.x:.2f}, '
            f'y={self.home_pose.pose.position.y:.2f}, yaw={math.degrees(yaw):.1f}deg'
        )

    # ============================================================
    # 상태 변경 / 출력
    # ============================================================
    def set_mode(self, mode: str):
        self.current_mode = mode
        msg = String()
        msg.data = mode
        self.state_pub.publish(msg)
        self.get_logger().info(f'state -> {mode}')

    # ============================================================
    # 상위 명령 콜백 (GUI에서 들어오는 high_level_cmd)
    # ============================================================
    def cmd_cb(self, msg: String):
        cmd = msg.data
        self.get_logger().info(f'high_level_cmd received: {cmd}')

        if cmd.startswith('GOTO_QR'):
            # ex) GOTO_QR1, GOTO_QR2 ...
            try:
                idx = int(cmd[-1])
            except ValueError:
                self.get_logger().error(f'GOTO_QR command parse error: {cmd}')
                return

            if idx not in self.qr_scan_poses:
                self.get_logger().error(f'Unknown QR index: {idx}')
                return

            self.current_qr = idx
            self.set_mode('GOTO_QR')
            self.send_goal_to_qr(idx)

        elif cmd == 'QR_SCAN_NEXT':
            # 현재 QR 스캔 완료 후 다음 QR 로 이동
            self.handle_qr_scan_next()

        elif cmd == 'SEQ_PATROL_START':
            # 정선 순찰 시작: QR1 → QR2 → QR3 → QR4 순서대로
            self.current_qr = 1
            self.last_scanned_qr = 0
            self.set_mode('SEQ_PATROL')
            self.send_goal_to_qr(1)

        elif cmd == 'RANDOM_PATROL_START':
            # 난선 순찰: 4개의 QR 중 랜덤하게 하나 선택해서 이동
            self.set_mode('RANDOM_PATROL')
            self.send_random_patrol_goal()

        elif cmd == 'RETURN_HOME':
            # 최초 위치로 복귀
            self.set_mode('RETURN_HOME')
            self.send_goal_home()

        elif cmd == 'BUZZER':
            # 부저 동작: 회전 + (나중에 실제 부저 제어면 추가)
            self.set_mode('BUZZER')
            self.start_buzzer_behavior()

        elif cmd == 'EMERGENCY_STOP':
            # 비상 정지
            self.set_mode('E_STOP')
            self.emergency_stop()

        elif cmd == 'HANDOVER':
            # 멀티 로봇 교대 (나중에 2대 쓸 때 사용)
            self.start_handover()

        else:
            self.get_logger().warn(f'Unknown command: {cmd}')

    # ============================================================
    # QR 순찰 / 다음 포인트
    # ============================================================
    def handle_qr_scan_next(self):
        # 실제 QR 인식은 별도 qr_scanner 노드에서 처리하고,
        # 여기서는 순찰 순서 관리만 담당하는 형태로 설계 가능
        self.get_logger().info(f'QR{self.current_qr} 스캔 완료 (논리상 처리)')
        self.last_scanned_qr = self.current_qr

        nxt = self.current_qr + 1
        if nxt > 4:
            nxt = 1
        self.current_qr = nxt

        if self.current_mode in ['SEQ_PATROL', 'IDLE']:
            self.send_goal_to_qr(nxt)

    # ============================================================
    # RANDOM PATROL
    # ============================================================
    def send_random_patrol_goal(self):
        idx = random.randint(1, 4)
        self.current_qr = idx
        self.get_logger().info(f'[RANDOM] 선택된 QR index = {idx}')
        self.send_goal_to_qr(idx)

    # ============================================================
    # RETURN HOME
    # ============================================================
    def send_goal_home(self):
        if self.home_pose is None:
            self.get_logger().warn(
                '[RETURN_HOME] home_pose가 아직 설정되지 않았습니다. '
                'odom 데이터가 먼저 한 번 들어와야 합니다.'
            )
            return

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        # home_pose 를 map 기준으로 저장했다고 가정
        pose.header.frame_id = 'map'
        pose.pose = self.home_pose.pose

        self.goal_pub.publish(pose)
        self.get_logger().info(
            f'[RETURN_HOME] home_pose 로 이동 명령 발행: '
            f'x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}'
        )

    # ============================================================
    # 목표 도착 콜백 (simple_navigator → goal_reached)
    # ============================================================
    def goal_reached_cb(self, msg: Bool):
        if not msg.data:
            return

        self.get_logger().info(
            f'[{self.robot_ns}] goal_reached, mode={self.current_mode}'
        )

        if self.current_mode in ['GOTO_QR', 'IDLE']:
            self.set_mode('IDLE')

        elif self.current_mode == 'SEQ_PATROL':
            # 정선 순찰 모드에서는 QR_SCAN_NEXT 또는 여기에서 바로 다음 QR 로 넘어가는 방식 중 선택 가능
            self.set_mode('IDLE')

        elif self.current_mode == 'RANDOM_PATROL':
            # 한 번 도착 후에는 일단 IDLE 로
            self.set_mode('IDLE')

        elif self.current_mode == 'RETURN_HOME':
            self.set_mode('IDLE')

        elif self.current_mode == 'BUZZER':
            self.set_mode('IDLE')

        elif self.current_mode in ['HANDOVER_GO_NEXT', 'TAKEOVER_GO_TO_LAST_QR',
                                   'WAIT_AFTER_HANDOVER']:
            # 멀티 로봇 시나리오 용 (나중에 확장)
            self.set_mode('IDLE')

    # ============================================================
    # QR 스캔 위치로 goal 전송
    # ============================================================
    def send_goal_to_qr(self, idx: int):
        if idx not in self.qr_scan_poses:
            self.get_logger().error(f'QR{idx} scan pose not defined')
            return

        x, y, yaw = self.qr_scan_poses[idx]

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'  # simple_navigator 와 map 기준으로 맞추기로 함

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_to_quat(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.goal_pub.publish(pose)
        self.get_logger().info(
            f'[BM] QR{idx} 스캔 위치로 이동 목표 발행: '
            f'x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.1f}deg'
        )

    # ============================================================
    # 교대 관련 (멀티 로봇 확장용 – 지금은 1대만 써도 문제 없음)
    # ============================================================
    def start_handover(self):
        # 현재는 1대만 사용해도 이 코드는 그냥 로그만 찍고 끝나므로 영향 없음
        if self.last_scanned_qr == 0:
            self.last_scanned_qr = self.current_qr

        msg = Int32()
        msg.data = self.last_scanned_qr
        self.handover_pub.publish(msg)
        self.get_logger().info(
            f'[HANDOVER] publish last_scanned_qr={self.last_scanned_qr}'
        )

    def handover_cb(self, msg: Int32):
        last_qr = msg.data
        self.get_logger().info(f'[HANDOVER] received last_qr={last_qr}')
        # 현재는 1대로만 사용하므로 실제 동작은 생략

    # ============================================================
    # 부저 / 비상정지
    # ============================================================
    def send_nav_cancel(self):
        msg = Bool()
        msg.data = True
        self.nav_cancel_pub.publish(msg)

    def start_buzzer_behavior(self):
        # 먼저 내비게이션 취소
        self.send_nav_cancel()

        twist = Twist()
        twist.angular.z = -1.0
        self.manual_vel_pub.publish(twist)
        self.get_logger().info('부저 동작: 제자리 회전 (수동 속도 발행)')

    def emergency_stop(self):
        # 내비게이션 취소 + 속도 0
        self.send_nav_cancel()

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.manual_vel_pub.publish(twist)
        self.get_logger().warn('EMERGENCY STOP: 속도 0')

    # ============================================================
    # 장애물 감지 이벤트
    # ============================================================
    def alarm_cb(self, msg: Bool):
        if msg.data:
            self.get_logger().warn(
                'Moving obstacle detected -> BUZZER behavior'
            )
            self.set_mode('BUZZER')
            self.start_buzzer_behavior()


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
