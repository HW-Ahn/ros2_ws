# 파일: ros2_ws/src/tb3_security_system/tb3_security_system/nodes/behavior_manager.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Twist

class BehaviorManager(Node):
    def set_mode(self, mode: str):
        self.set_mode = mode
        msg = String()
        msg.data = mode
        self.state_pub.publish(msg)
        self.get_logger().info(f'state -> {mode}')
    def __init__(self):
        super().__init__('behavior_manager')

        #상태 퍼블리시 추가
        self.state_pub = self.create_publisher(String, 'state', 10)
        self.set_mode('IDLE')  # 초기 상태

        # 네임스페이스 내 토픽
        self.cmd_sub = self.create_subscription(String, 'high_level_cmd', self.cmd_cb, 10)
        self.goal_pub = self.create_publisher(PoseStamped, 'cmd_goal', 10)
        self.manual_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)  # nav2가 쓰는 cmd_vel
        self.alarm_sub = self.create_subscription(Bool, 'alarm_event', self.alarm_cb, 10)

        # 상태 변수
        self.set_mode = ('IDLE')
        self.current_qr = 1
        self.home_pose = None  # 최초 위치를 나중에 저장
        self.get_logger().info('BehaviorManager started')

        # TODO: home_pose를 /amcl_pose 또는 /odom에서 처음 한 번 읽어들여 저장하는 로직을 추가할 수 있습니다.

    def cmd_cb(self, msg: String):
        cmd = msg.data
        self.get_logger().info(f'high_level_cmd received: {cmd}')

        # 모든 새 명령은 우선 기존 동작을 중지하는 방향으로 설계
        if cmd.startswith('GOTO_QR'):
            # 예: GOTO_QR1
            idx = int(cmd[-1])
            self.set_mode = ('GOTO_QR')
            self.current_qr = idx
            self.send_goal_to_qr(idx)
        elif cmd == 'QR_SCAN_NEXT':
            self.handle_qr_scan_next()
        elif cmd == 'SEQ_PATROL_START':
            self.set_mode = ('SEQ_PATROL')
            self.current_qr = 1
            self.send_goal_to_qr(1)
        elif cmd == 'RANDOM_PATROL_START':
            self.set_mode = ('RANDOM_PATROL')
            # TODO: 랜덤 목표 좌표 생성 후 send_goal_to_random()
        elif cmd == 'RETURN_HOME':
            self.set_mode = ('RETURN_HOME')
            # TODO: home_pose로 goal 전송
        elif cmd == 'BUZZER':
            self.set_mode = ('BUZZER')
            self.start_buzzer_behavior()
        elif cmd == 'EMERGENCY_STOP':
            self.set_mode = ('E_STOP')
            self.emergency_stop()
        else:
            self.get_logger().warn(f'Unknown command: {cmd}')

    def alarm_cb(self, msg: Bool):
        if msg.data:
            self.get_logger().warn('Moving obstacle detected -> BUZZER behavior')
            # 이동 장애물 감지시 강제로 부저 모드로 전환 (요구사항 2,7)
            self.set_mode = ('BUZZER')
            self.start_buzzer_behavior()

    # ------------------------
    # 세부 동작 메서드
    # ------------------------
    def send_goal_to_qr(self, idx: int):
        # 실제 QR 좌표는 GUI의 ControlBridge와 중복 정의하지 않고, parameter나 공용 util로 빼는 것이 이상적입니다.
        # 여기서는 단순 예시로 logger만 남깁니다.
        self.get_logger().info(f'[BM] QR{idx} 위치로 이동 목표 요청 (실제 goal은 ControlBridge에서 보내도 되고, 여기서 보내도 됩니다)')

    def handle_qr_scan_next(self):
        self.get_logger().info(f'QR{self.current_qr} 스캔 완료 (시뮬레이션: 터미널 출력만)')
        # 다음 번호로
        nxt = self.current_qr + 1
        if nxt > 4:
            nxt = 1
        self.current_qr = nxt
        if self.set_mode in ['SEQ_PATROL', 'IDLE']:
            # 다음 포인트로 이동
            self.send_goal_to_qr(nxt)

    def start_buzzer_behavior(self):
        # 단순하게 한 번 회전 명령만 내보내는 예시
        twist = Twist()
        twist.angular.z = -1.0  # 시계 방향
        self.manual_vel_pub.publish(twist)
        self.get_logger().info('부저 동작: 제자리 회전 + (실제 부저 소리는 별도 노드/PC에서 구현 가능)')

    def emergency_stop(self):
        # 속도 0
        twist = Twist()
        self.manual_vel_pub.publish(twist)
        self.get_logger().warn('EMERGENCY STOP: 속도 0, 추가로 nav2 goal cancel 로직 필요')


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
