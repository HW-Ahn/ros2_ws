# 파일: tb3_security_system/nodes/behavior_manager.py

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int32
from geometry_msgs.msg import PoseStamped, Twist

from turtlebot3_msgs.srv import Sound  # 부저 서비스


def yaw_to_quat(yaw: float):
    """2D yaw(라디안) -> 쿼터니언 x,y,z,w (roll/pitch = 0 가정)."""
    half = yaw * 0.5
    cz = math.cos(half)
    sz = math.sin(half)
    return (0.0, 0.0, sz, cz)


class BehaviorManager(Node):
    def __init__(self):
        super().__init__('behavior_manager')

        # 네임스페이스 확인 (예: /tb3_1, /tb3_2)
        ns = self.get_namespace().strip('/')
        self.robot_ns = ns if ns else 'tb3_1'
        self.is_primary = (self.robot_ns == 'tb3_1')  # tb3_1이 기본 주 순찰 로봇

        # 상태 변수
        self.current_mode = 'IDLE'
        self.current_qr = 1
        self.last_scanned_qr = 0
        self.home_pose = None  # TODO: odom/map에서 초기 위치 저장하는 로직 추가 가능

        # ---------------- QR 스캔 위치 (이미 최종 확정 값 사용) ----------------
        # A안: 사용자가 알려준 실제 좌표 (odom 기준), yaw는 일단 0.0으로 통일
        self.qr_scan_poses = {
            1: ( 1.4, -1.5, -math.pi / 2.0),
            2: ( 4.7, -1.15, -math.pi / 2.2),
            3: ( 4.2,  2.1,  math.pi / 2.0),
            4: ( 0.7, 2.1,  math.pi / 2.0),
        }

        # ---------------- 퍼블리셔 / 서브스크립션 ----------------
        # GUI에서 보는 상태
        self.state_pub = self.create_publisher(String, 'state', 10)

        # simple_navigator 가 구독하는 목표 토픽
        # simple_navigator.py 의 create_subscription('cmd_goal', ...)에 맞춤
        self.goal_pub = self.create_publisher(PoseStamped, 'cmd_goal', 10)

        # 수동 조종 속도 (simple_navigator 에서 cmd_vel_manual를 받아 사용한다고 가정)
        self.manual_vel_pub = self.create_publisher(Twist, 'cmd_vel_manual', 10)

        # 이벤트/명령/내비 상태
        self.alarm_sub = self.create_subscription(Bool, 'alarm_event', self.alarm_cb, 10)
        self.cmd_sub = self.create_subscription(String, 'high_level_cmd', self.cmd_cb, 10)
        self.goal_reached_sub = self.create_subscription(Bool, 'goal_reached', self.goal_reached_cb, 10)

        # 교대 정보 공유 토픽
        self.handover_pub = self.create_publisher(Int32, '/patrol/handover', 10)
        self.handover_sub = self.create_subscription(Int32, '/patrol/handover', self.handover_cb, 10)

        # 내비게이션 취소 (simple_navigator용)
        # simple_navigator.py 에서 nav_cancel 구독하는 부분에 맞춤
        self.nav_cancel_pub = self.create_publisher(Bool, 'nav_cancel', 10)

        # ---------------- 부저 서비스 클라이언트 ----------------
        # 이 노드는 PushRosNamespace 로 /tb3_1 아래서 돌기 때문에
        # 여기서 서비스 이름을 'sound' 로 만들면 실제 이름은 /tb3_1/sound 가 됨.
        self.sound_client = self.create_client(Sound, 'sound')

        # ---------------- 부저 회전용 타이머 ----------------
        self.buzzer_timer = None
        self.buzzer_ticks = 0       # 회전 횟수 카운트
        self.buzzer_max_ticks = 30  # 0.1초 × 30 = 3초 동안 회전

        self.set_mode('IDLE')
        self.get_logger().info(
            f'BehaviorManager started for {self.robot_ns}, primary={self.is_primary}'
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
    # 상위 명령 콜백 (GUI에서 오는 high_level_cmd)
    # ============================================================
    def cmd_cb(self, msg: String):
        cmd = msg.data
        self.get_logger().info(f'high_level_cmd received: {cmd}')

        if cmd.startswith('GOTO_QR'):
            idx = int(cmd[-1])
            self.current_qr = idx
            self.set_mode('GOTO_QR')
            self.send_goal_to_qr(idx)

        elif cmd == 'QR_SCAN_NEXT':
            self.handle_qr_scan_next()

        elif cmd == 'SEQ_PATROL_START':
            # 정선순찰 시작: 주 로봇만 시작, 보조 로봇은 대기
            if self.is_primary:
                self.current_qr = 1
                self.last_scanned_qr = 0
                self.set_mode('SEQ_PATROL')
                self.send_goal_to_qr(1)
            else:
                self.get_logger().info('Secondary robot: waiting for handover.')

        elif cmd == 'RANDOM_PATROL_START':
            self.set_mode('RANDOM_PATROL')
            # TODO: 랜덤 좌표 생성해서 self.send_goal_to_random() 구현 가능

        elif cmd == 'RETURN_HOME':
            self.set_mode('RETURN_HOME')
            # TODO: home_pose 로 goal 전송 (home_pose 저장 로직 필요)

        elif cmd == 'BUZZER':
            self.set_mode('BUZZER')
            self.start_buzzer_behavior()

        elif cmd == 'EMERGENCY_STOP':
            self.set_mode('E_STOP')
            self.emergency_stop()

        elif cmd == 'HANDOVER':
            # 교대 명령: 주 로봇만 처리
            if self.is_primary:
                self.start_handover()
            else:
                self.get_logger().info('Secondary robot: HANDOVER cmd ignored.')

        else:
            self.get_logger().warn(f'Unknown command: {cmd}')

    # ============================================================
    # 교대 관련 로직
    # ============================================================
    def start_handover(self):
        if self.last_scanned_qr == 0:
            self.last_scanned_qr = self.current_qr

        msg = Int32()
        msg.data = self.last_scanned_qr
        self.handover_pub.publish(msg)
        self.get_logger().info(
            f'[HANDOVER] publish last_scanned_qr={self.last_scanned_qr}'
        )

        # 자기 자신은 다음 QR로 이동
        next_qr = self.last_scanned_qr + 1
        if next_qr > 4:
            next_qr = 1
        self.current_qr = next_qr
        self.set_mode('HANDOVER_GO_NEXT')
        self.send_goal_to_qr(next_qr)

    def handover_cb(self, msg: Int32):
        last_qr = msg.data
        self.get_logger().info(f'[HANDOVER] received last_qr={last_qr}')

        if self.is_primary:
            self.get_logger().info('Primary robot: handover msg received (no action).')
            return

        self.current_qr = last_qr
        self.last_scanned_qr = last_qr
        self.set_mode('TAKEOVER_GO_TO_LAST_QR')
        self.send_goal_to_qr(last_qr)

    # ============================================================
    # QR 순찰 / 다음 포인트
    # ============================================================
    def handle_qr_scan_next(self):
        self.get_logger().info(
            f'QR{self.current_qr} 스캔 완료 (시뮬레이션: 로그만)'
        )
        self.last_scanned_qr = self.current_qr

        nxt = self.current_qr + 1
        if nxt > 4:
            nxt = 1
        self.current_qr = nxt

        if self.current_mode in ['SEQ_PATROL', 'IDLE']:
            self.send_goal_to_qr(nxt)

    # ============================================================
    # 목표 도착 콜백 (simple_navigator 가 /goal_reached 발행한다고 가정)
    # ============================================================
    def goal_reached_cb(self, msg: Bool):
        if not msg.data:
            return

        self.get_logger().info(
            f'[{self.robot_ns}] goal_reached, mode={self.current_mode}'
        )

        if self.current_mode == 'GOTO_QR':
            self.set_mode('IDLE')

        elif self.current_mode == 'SEQ_PATROL':
            self.set_mode('IDLE')

        elif self.current_mode == 'HANDOVER_GO_NEXT':
            self.is_primary = False
            self.set_mode('WAIT_AFTER_HANDOVER')
            # TODO: RETURN_HOME 구현 시 여기에서 home으로 이동 명령

        elif self.current_mode == 'TAKEOVER_GO_TO_LAST_QR':
            self.is_primary = True
            self.set_mode('SEQ_PATROL')
            nxt = self.current_qr + 1
            if nxt > 4:
                nxt = 1
            self.current_qr = nxt
            self.send_goal_to_qr(nxt)

        elif self.current_mode == 'RETURN_HOME':
            self.set_mode('IDLE')

        elif self.current_mode == 'BUZZER':
            self.set_mode('IDLE')

    # ============================================================
    # 목표 전송 (QR 스캔 위치 + yaw)  -- A안: frame_id = 'odom'
    # ============================================================
    def send_goal_to_qr(self, idx: int):
        if idx not in self.qr_scan_poses:
            self.get_logger().error(f'QR{idx} scan pose not defined')
            return

        x, y, yaw = self.qr_scan_poses[idx]

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'odom'  # A안: odom 고정

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
            f'x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.1f}deg (frame=odom)'
        )

    # ============================================================
    # 내비게이션 취소
    # ============================================================
    def send_nav_cancel(self):
        msg = Bool()
        msg.data = True
        self.nav_cancel_pub.publish(msg)

    # ============================================================
    # 부저 동작: 내비 취소 + 3초 회전(-1.0rad/s) + 사운드
    # ============================================================
    def start_buzzer_behavior(self):
        # 1) 내비게이션 취소
        self.send_nav_cancel()

        # 2) 사운드 서비스 호출
        self.play_buzzer_sound(3)

        # 3) 제자리 회전 타이머 시작 (각속도 -1.0 rad/s)
        self.buzzer_ticks = 0
        if self.buzzer_timer is None:
            self.buzzer_timer = self.create_timer(0.1, self.buzzer_timer_cb)
        self.get_logger().info('부저 동작 시작: 제자리 회전 + 사운드')

    def buzzer_timer_cb(self):
        # 0.1s 마다 호출, 총 buzzer_max_ticks 번 회전 명령
        if self.buzzer_ticks < self.buzzer_max_ticks:
            twist = Twist()
            twist.angular.z = -1.0  # A안: -1.0 rad/s
            self.manual_vel_pub.publish(twist)
            self.buzzer_ticks += 1
        else:
            # 회전 종료
            if self.buzzer_timer is not None:
                self.buzzer_timer.cancel()
                self.buzzer_timer = None

            twist = Twist()  # 0 속도
            self.manual_vel_pub.publish(twist)
            self.get_logger().info('부저 동작 종료: 속도 0')
            if self.current_mode == 'BUZZER':
                self.set_mode('IDLE')

    def play_buzzer_sound(self, value: int = 3):
        """turtlebot3_msgs/srv/Sound 서비스 호출해서 부저 울리기."""
        if not self.sound_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn('Sound 서비스가 준비되지 않았습니다.')
            return

        req = Sound.Request()
        req.value = int(value)
        future = self.sound_client.call_async(req)

        def _done_cb(fut):
            if fut.result() is not None:
                self.get_logger().info('Sound 서비스 호출 성공')
            else:
                self.get_logger().warn('Sound 서비스 호출 실패')

        future.add_done_callback(_done_cb)

    # ============================================================
    # 비상정지
    # ============================================================
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
            self.get_logger().warn('Moving obstacle detected -> BUZZER behavior')
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
