# 파일: tb3_security_system/nodes/behavior_manager.py

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int32
from geometry_msgs.msg import PoseStamped, Twist


def yaw_to_quat(yaw: float):
    """2D yaw(라디안) -> 쿼터니언 x,y,z,w (롤/피치는 0 가정)."""
    half = yaw * 0.5
    cz = math.cos(half)
    sz = math.sin(half)
    # 롤=피치=0일 때 yaw만 반영
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
        self.home_pose = None  # TODO: 나중에 odom에서 저장하는 로직 추가 가능

        # QR 박스 좌표 및 방향 정보
        # face: 'N'이면 QR 면이 북쪽(+y), 'S'면 남쪽(-y)
        self.qr_boxes = {
            1: {'x': -1.8, 'y': -2.3, 'face': 'N'},
            2: {'x':  1.8, 'y': -2.3, 'face': 'N'},
            3: {'x':  1.8, 'y':  2.3, 'face': 'S'},
            4: {'x': -1.8, 'y':  2.3, 'face': 'S'},
        }

        # QR 스캔 위치 계산 (0.4m 앞에서 정면으로)
        self.scan_offset = 0.4  # 박스에서 떨어질 거리
        self.qr_scan_poses = {}  # idx -> (scan_x, scan_y, scan_yaw)

        for idx, info in self.qr_boxes.items():
            bx = info['x']
            by = info['y']
            face = info['face']
            if face == 'N':
                # 박스가 북쪽(+y)을 보고 있으니,
                # 로봇은 남쪽(-y)에 서서 '북쪽을 바라보는' 것이 원래인데
                # 실제 방향이 반대로 느껴졌으므로 yaw 부호를 반대로 적용
                scan_x = bx
                scan_y = by - self.scan_offset
                scan_yaw = -math.pi / 2.0  # 이전: +pi/2, 이제 반대로
            elif face == 'S':
                # 박스가 남쪽(-y)을 보고 있음 → 로봇은 북쪽(+y)에 서서 남쪽을 봐야 하는데
                # 마찬가지로 yaw 부호를 반대로 적용
                scan_x = bx
                scan_y = by + self.scan_offset
                scan_yaw = +math.pi / 2.0  # 이전: -pi/2, 이제 반대로
            else:
                scan_x = bx
                scan_y = by
                scan_yaw = 0.0
            self.qr_scan_poses[idx] = (scan_x, scan_y, scan_yaw)

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

        # 내비게이션 취소 (simple_navigator용)
        self.nav_cancel_pub = self.create_publisher(Bool, 'nav_cancel', 10)

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
        last_qr = msg.data
        self.get_logger().info(f'[HANDOVER] received last_qr={last_qr}')

        if self.is_primary:
            self.get_logger().info('Primary robot: handover msg received (no action).')
            return

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

    # ------------------------
    # 목표 전송 (QR 스캔 위치 + 정면 yaw)
    # ------------------------
    def send_goal_to_qr(self, idx: int):
        if idx not in self.qr_scan_poses:
            self.get_logger().error(f'QR{idx} scan pose not defined')
            return

        x, y, yaw = self.qr_scan_poses[idx]

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'odom'

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
            f'[BM] QR{idx} 스캔 위치로 이동 목표 발행: x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.1f}deg'
        )

    # ------------------------
    # 부저 / 비상정지
    # ------------------------
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
        self.get_logger().info('부저 동작: 제자리 회전')

    def emergency_stop(self):
        # 내비게이션 취소 + 속도 0
        self.send_nav_cancel()

        twist = Twist()
        # 기본값이 0,0 이므로 굳이 안 채워도 되지만 명시적으로 남김
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.manual_vel_pub.publish(twist)
        self.get_logger().warn('EMERGENCY STOP: 속도 0')

    # ------------------------
    # 장애물 감지 이벤트 (별도 노드에서 alarm_event를 보낸다면)
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
