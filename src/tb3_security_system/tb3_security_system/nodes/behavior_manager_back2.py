# 파일: tb3_security_system/nodes/behavior_manager.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int32
from geometry_msgs.msg import PoseStamped, Twist


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
        self.home_pose = None  # TODO: 처음 odom에서 받아서 저장하는 로직 추가 가능

        # QR 좌표 (예시 값 - 필요시 수정)
        # frame_id='odom' 기준 좌표라고 가정
        self.qr_poses = {
            1: (-1.8, -2.2),
            2: (1.8, -2.2),
            3: (1.8, 2.2),
            4: (-1.8, 2.2),

        }

        # 퍼블리셔 / 서브스크립션
        self.state_pub = self.create_publisher(String, 'state', 10)
        self.goal_pub = self.create_publisher(PoseStamped, 'cmd_goal', 10)
        self.manual_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.alarm_sub = self.create_subscription(Bool, 'alarm_event', self.alarm_cb, 10)
        self.cmd_sub = self.create_subscription(String, 'high_level_cmd', self.cmd_cb, 10)
        self.goal_reached_sub = self.create_subscription(Bool, 'goal_reached', self.goal_reached_cb, 10)

        # 교대 정보 공유 토픽
        self.handover_pub = self.create_publisher(Int32, '/patrol/handover', 10)
        self.handover_sub = self.create_subscription(Int32, '/patrol/handover', self.handover_cb, 10)

        self.set_mode('IDLE')
        self.get_logger().info(f'BehaviorManager started for {self.robot_ns}, primary={self.is_primary}')

    # ------------------------
    # 상태 변경 / 출력
    # ------------------------
    def set_mode(self, mode: str):
        self.current_mode = mode
        msg = String()
        msg.data = mode
        self.state_pub.publish(msg)
        self.get_logger().info(f'state -> {mode}')

    # ------------------------
    # 상위 명령 콜백
    # ------------------------
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
            # TODO: 랜덤 좌표 생성해서 self.send_goal_to_random()

        elif cmd == 'RETURN_HOME':
            self.set_mode('RETURN_HOME')
            # TODO: home_pose로 goal 전송

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

    # ------------------------
    # 교대 관련 로직
    # ------------------------
    def start_handover(self):
        # 마지막으로 스캔한 QR이 없으면 현재 QR을 기준으로 한다고 가정
        if self.last_scanned_qr == 0:
            self.last_scanned_qr = self.current_qr

        msg = Int32()
        msg.data = self.last_scanned_qr
        self.handover_pub.publish(msg)
        self.get_logger().info(f'[HANDOVER] publish last_scanned_qr={self.last_scanned_qr}')

        # 자기 자신은 다음 QR로 이동
        next_qr = self.last_scanned_qr + 1
        if next_qr > 4:
            next_qr = 1
        self.current_qr = next_qr
        self.set_mode('HANDOVER_GO_NEXT')
        self.send_goal_to_qr(next_qr)

    def handover_cb(self, msg: Int32):
        # 교대 브로드캐스트 수신
        last_qr = msg.data
        self.get_logger().info(f'[HANDOVER] received last_qr={last_qr}')

        # 주 로봇이 보내는 것이므로, 보조 로봇만 동작
        if self.is_primary:
            self.get_logger().info('Primary robot: handover msg received (no action).')
            return

        # 보조 로봇은 last_qr 위치로 이동 후 순찰 이어가기
        self.current_qr = last_qr
        self.last_scanned_qr = last_qr
        self.set_mode('TAKEOVER_GO_TO_LAST_QR')
        self.send_goal_to_qr(last_qr)

    # ------------------------
    # QR 순찰 / 다음 포인트
    # ------------------------
    def handle_qr_scan_next(self):
        self.get_logger().info(f'QR{self.current_qr} 스캔 완료 (시뮬레이션: 로그만)')
        self.last_scanned_qr = self.current_qr

        nxt = self.current_qr + 1
        if nxt > 4:
            nxt = 1
        self.current_qr = nxt

        if self.current_mode in ['SEQ_PATROL', 'IDLE']:
            self.send_goal_to_qr(nxt)

    # ------------------------
    # 목표 도착 콜백
    # ------------------------
    def goal_reached_cb(self, msg: Bool):
        if not msg.data:
            return

        self.get_logger().info(f'[{self.robot_ns}] goal_reached, mode={self.current_mode}')

        if self.current_mode == 'GOTO_QR':
            self.set_mode('IDLE')

        elif self.current_mode == 'SEQ_PATROL':
            # SEQ_PATROL에서는 보통 QR_SCAN_NEXT 명령을 GUI에서 눌러서 다음으로 이동
            self.set_mode('IDLE')

        elif self.current_mode == 'HANDOVER_GO_NEXT':
            # 주 로봇: 다음 QR 도착 → 복귀하거나 대기
            self.is_primary = False
            self.set_mode('WAIT_AFTER_HANDOVER')
            # TODO: RETURN_HOME 구현 시 여기서 home으로 이동 명령

        elif self.current_mode == 'TAKEOVER_GO_TO_LAST_QR':
            # 보조 로봇: 마지막 QR 도착 → 순찰 이어가기
            self.is_primary = True
            self.set_mode('SEQ_PATROL')
            # 마지막 스캔된 QR에서 다음 QR로 이동
            nxt = self.current_qr + 1
            if nxt > 4:
                nxt = 1
            self.current_qr = nxt
            self.send_goal_to_qr(nxt)

        elif self.current_mode == 'RETURN_HOME':
            self.set_mode('IDLE')

    # ------------------------
    # 목표 전송
    # ------------------------
    def send_goal_to_qr(self, idx: int):
        if idx not in self.qr_poses:
            self.get_logger().error(f'QR{idx} pose not defined')
            return

        x, y = self.qr_poses[idx]
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'odom'   # SimpleNavigator 기준

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0   # 단순 yaw=0

        self.goal_pub.publish(pose)
        self.get_logger().info(f'[BM] QR{idx}로 이동 목표 발행: x={x:.2f}, y={y:.2f}')

    # ------------------------
    # 부저 / 비상정지
    # ------------------------
    def start_buzzer_behavior(self):
        twist = Twist()
        twist.angular.z = -1.0
        self.manual_vel_pub.publish(twist)
        self.get_logger().info('부저 동작: 제자리 회전')

    def emergency_stop(self):
        twist = Twist()
        self.manual_vel_pub.publish(twist)
        self.get_logger().warn('EMERGENCY STOP: 속도 0')

    # ------------------------
    # 장애물 감지 이벤트 (별도 로직이 있다면)
    # ------------------------
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
