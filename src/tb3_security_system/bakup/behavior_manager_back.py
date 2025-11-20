import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Twist

QR_POSITIONS = {
    1: (-1.8, -2.2),
    2: (-1.8, 2.2),
    3: (1.8, 2.2),
    4: (1.8, -2.2),
}


class BehaviorManager(Node):
    def __init__(self):
        super().__init__('behavior_manager')

        # ---------------------------
        # 상태 변수 (필수 수정된 부분)
        # ---------------------------
        self.current_mode = 'IDLE'       # 초기 상태
        self.current_qr = 1              # 현재 목표 QR
        self.home_pose = None            # 초기 위치 저장용 (필요 시 사용)

        # ---------------------------
        # ROS2 Publisher / Subscriber
        # ---------------------------
        self.state_pub = self.create_publisher(String, 'state', 10)
        self.goal_pub = self.create_publisher(PoseStamped, 'cmd_goal', 10)
        self.manual_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.cmd_sub = self.create_subscription(
            String,
            'high_level_cmd',
            self.cmd_callback,
            10
        )

        self.alarm_sub = self.create_subscription(
            Bool,
            'alarm_event',
            self.alarm_callback,
            10
        )

        # 초기 상태 송신
        self.publish_state()
        self.get_logger().info('BehaviorManager successfully started.')

    # ---------------------------
    # 상태 전환 함수
    # ---------------------------
    def set_mode(self, mode: str):
        """현재 모드 값을 갱신하고 퍼블리시"""
        self.current_mode = mode
        self.publish_state()
        self.get_logger().info(f'Mode changed → {self.current_mode}')

    def publish_state(self):
        msg = String()
        msg.data = self.current_mode
        self.state_pub.publish(msg)

    # ---------------------------
    # 고레벨 명령 처리
    # ---------------------------
    def cmd_callback(self, msg: String):
        cmd = msg.data
        self.get_logger().info(f'[BM] Received high_level_cmd : {cmd}')

        if cmd.startswith('GOTO_QR'):
            idx = int(cmd[-1])
            self.set_mode('GOTO_QR')
            self.current_qr = idx
            self.send_goal_to_qr(idx)

        elif cmd == 'QR_SCAN_NEXT':
            self.handle_qr_scan_next()

        elif cmd == 'SEQ_PATROL_START':
            self.set_mode('SEQ_PATROL')
            self.current_qr = 1
            self.send_goal_to_qr(1)

        elif cmd == 'RANDOM_PATROL_START':
            self.set_mode('RANDOM_PATROL')
            # TODO: 랜덤 위치 생성 후 이동
            self.get_logger().info('[BM] Random patrol not implemented yet')

        elif cmd == 'RETURN_HOME':
            self.set_mode('RETURN_HOME')
            # TODO: home_pose를 사용해서 goal 전송
            self.get_logger().info('[BM] Return Home requested.')

        elif cmd == 'BUZZER':
            self.set_mode('BUZZER')
            self.start_buzzer_behavior()

        elif cmd == 'EMERGENCY_STOP':
            self.set_mode('E_STOP')
            self.emergency_stop()

        else:
            self.get_logger().warn(f'[BM] Unknown command: {cmd}')

    # ---------------------------
    # 알람 이벤트 처리
    # ---------------------------
    def alarm_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().warn('[BM] Moving obstacle detected → BUZZER MODE')
            self.set_mode('BUZZER')
            self.start_buzzer_behavior()

    # ---------------------------
    # 세부 동작 구현부
    # ---------------------------
    def send_goal_to_qr(self, idx: int):
        if idx not in QR_POSITIONS:
            self.get_logger().warn(f'[BM] QR{idx} 위치 정보가 정의되어 있지 않습니다.')
            return

        x, y = QR_POSITIONS[idx]

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'  # 필요에 따라 'odom' 등으로 변경

        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.125

        # 방향은 일단 기본(0도)로, 회전 없이 설정
        goal.pose.orientation.w = 1.0

        self.goal_pub.publish(goal)
        self.get_logger().info(f'[BM] QR{idx}로 이동 목표 발행: x={x:.2f}, y={y:.2f}')

    def handle_qr_scan_next(self):
        self.get_logger().info(f'[BM] QR{self.current_qr} scan complete')

        next_qr = self.current_qr + 1
        if next_qr > 4:
            next_qr = 1

        self.current_qr = next_qr

        if self.current_mode in ['SEQ_PATROL', 'IDLE']:
            self.send_goal_to_qr(next_qr)

    def start_buzzer_behavior(self):
        twist = Twist()
        twist.angular.z = -1.0
        self.manual_vel_pub.publish(twist)
        self.get_logger().info('[BM] BUZZER behavior running (spin in place)')

    def emergency_stop(self):
        twist = Twist()
        self.manual_vel_pub.publish(twist)
        self.get_logger().warn('[BM] EMERGENCY STOP – Velocity zeroed.')

def main(args=None):
    rclpy.init(args=args)
    node = BehaviorManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
