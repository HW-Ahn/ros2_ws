# 파일: tb3_security_system/nodes/behavior_manager.py

import math
import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int32
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from turtlebot3_msgs.srv import Sound


def yaw_to_quat(yaw: float):
    """2D yaw -> 쿼터니언 (x,y,z,w)"""
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class BehaviorManager(Node):
    def __init__(self):
        super().__init__("behavior_manager")

        # 네임스페이스 (/tb3_1, /tb3_2 ...)
        ns = self.get_namespace().strip('/')   # ★ 오타 수정(stㅁrip -> strip)
        self.robot_ns = ns if ns else "tb3_1"
        self.is_primary = (self.robot_ns == "tb3_1")

        # 상태 변수
        self.current_mode = "IDLE"
        self.current_qr = 1
        self.last_scanned_qr = 0

        # home_pose: 최초 odom을 자동으로 저장
        self.home_pose = None

        # ---------- QR 목표 위치 (현재 A안, odom 기준) ----------
        # 1,4 좌표는 수정된 값 사용
        self.qr_scan_poses = {
            1: (1.0, -1.7, -math.pi / 2.0),  # 수정된 QR1
            2: (4.5, -1.5, -math.pi / 2.0),
            3: (4.4,  2.0,  math.pi / 2.0),
            4: (1.0,  2.0,  math.pi / 2.0),  # 수정된 QR4
        }

        # ---------- Publisher / Subscriber ----------
        self.state_pub = self.create_publisher(String, 'state', 10)
        self.goal_pub = self.create_publisher(PoseStamped, 'cmd_goal', 10)
        self.manual_pub = self.create_publisher(Twist, 'cmd_vel_manual', 10)
        self.nav_cancel_pub = self.create_publisher(Bool, 'nav_cancel', 10)
        self.handover_pub = self.create_publisher(Int32, '/patrol/handover', 10)

        self.cmd_sub = self.create_subscription(
            String, 'high_level_cmd', self.cmd_cb, 10
        )
        self.goal_reached_sub = self.create_subscription(
            Bool, 'goal_reached', self.goal_reached_cb, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_cb, 10
        )
        self.handover_sub = self.create_subscription(
            Int32, '/patrol/handover', self.handover_cb, 10
        )
        self.alarm_sub = self.create_subscription(
            Bool, 'alarm_event', self.alarm_cb, 10
        )

        # ---------- 부저 서비스 클라이언트 ----------
        # namespace 안에서 'sound' 이므로 실제 서비스 이름은 /tb3_1/sound
        self.sound_client = self.create_client(Sound, 'sound')

        # ---------- 부저 반복 + 회전용 변수 ----------
        self.buzzer_timer = None
        self.rotation_accum = 0.0
        self.last_yaw = None
        self.current_yaw = 0.0

        self.set_mode("IDLE")
        self.get_logger().info(f"BehaviorManager started for {self.robot_ns}")

    # ============================================================
    # 공용 유틸
    # ============================================================
    def set_mode(self, mode: str):
        self.current_mode = mode
        msg = String()
        msg.data = mode
        self.state_pub.publish(msg)
        self.get_logger().info(f"[BM] state -> {mode}")

    def send_nav_cancel(self):
        msg = Bool()
        msg.data = True
        self.nav_cancel_pub.publish(msg)

    # ============================================================
    # odom 콜백 (home_pose 저장 + yaw 추적)
    # ============================================================
    def odom_cb(self, msg: Odometry):
        # 최초 한 번 home_pose 저장
        if self.home_pose is None:
            self.home_pose = msg.pose.pose
            self.get_logger().info("*** Home Pose (odom) 저장 완료 ***")

        ori = msg.pose.pose.orientation
        _, _, yaw = self.quat_to_yaw(ori.x, ori.y, ori.z, ori.w)
        if self.last_yaw is None:
            self.last_yaw = yaw
        self.current_yaw = yaw

    def quat_to_yaw(self, x, y, z, w):
        siny = 2.0 * (w * z + x * y)
        cosy = 1.0 - 2.0 * (y * y + z * z)
        return (0.0, 0.0, math.atan2(siny, cosy))

    # ============================================================
    # 상위 명령 수신
    # ============================================================
    def cmd_cb(self, msg: String):
        cmd = msg.data
        self.get_logger().info(f"[BM] CMD: {cmd}")

        if cmd.startswith("GOTO_QR"):
            idx = int(cmd[-1])
            self.current_qr = idx
            self.send_goal_to_qr(idx)
            self.set_mode("GOTO_QR")

        elif cmd == "SEQ_PATROL_START":
            # 1번부터 순차순찰 시작, 계속 QR1~4 순환
            self.current_qr = 1
            self.set_mode("SEQ_PATROL")
            self.send_goal_to_qr(1)

        elif cmd == "RANDOM_PATROL_START":
            # 아무 QR이나 랜덤으로 계속 순찰
            self.set_mode("RANDOM_PATROL")
            self.current_qr = random.randint(1, 4)
            self.send_goal_to_qr(self.current_qr)

        elif cmd == "RETURN_HOME":
            self.handle_return_home()

        elif cmd == "BUZZER":
            self.start_buzzer_rotation()

        elif cmd == "EMERGENCY_STOP":
            self.emergency_stop()

        elif cmd == "HANDOVER":
            # (지금은 1대만 쓰고 있으니, 필요시 확장)
            self.start_handover()

        else:
            self.get_logger().warn(f"Unknown CMD: {cmd}")

    # ============================================================
    # QR 이동 목표 전송
    # ============================================================
    def send_goal_to_qr(self, idx):
        if idx not in self.qr_scan_poses:
            self.get_logger().error(f"QR index {idx} not defined")
            return

        x, y, yaw = self.qr_scan_poses[idx]

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "odom"     # A안: odom 기준 유지

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
            f"[BM] 이동 목표 QR{idx}: x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.1f}deg"
        )

    # ============================================================
    # RETURN_HOME
    # ============================================================
    def handle_return_home(self):
        if self.home_pose is None:
            self.get_logger().warn("★ home_pose가 아직 저장되지 않았습니다(odom 미수신).")
            return

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "odom"
        pose.pose = self.home_pose

        self.goal_pub.publish(pose)
        self.set_mode("RETURN_HOME")
        self.get_logger().info("[BM] RETURN_HOME 시작")

    # ============================================================
    # 부저 반복 + 1바퀴 회전
    # ============================================================
    def start_buzzer_rotation(self):
        self.send_nav_cancel()

        self.rotation_accum = 0.0
        self.last_yaw = None
        self.set_mode("BUZZER")

        # 0.1초 주기로 반복
        if self.buzzer_timer is not None:
            self.buzzer_timer.cancel()
        self.buzzer_timer = self.create_timer(0.1, self.buzzer_loop)
        self.get_logger().info("[BM] BUZZER 회전 시작")

    def buzzer_loop(self):
        # 각속도 유지 (제자리 회전)
        twist = Twist()
        twist.angular.z = -1.0
        self.manual_pub.publish(twist)

        # 사운드 호출
        self.call_buzzer_sound(3)

        # 회전량 누적
        try:
            dyaw = abs(self.current_yaw - self.last_yaw)
            if dyaw > math.pi:
                dyaw = (2 * math.pi) - dyaw
            self.rotation_accum += dyaw
        except Exception:
            pass

        self.last_yaw = self.current_yaw

        # 360도(2π) 회전 완료 → 종료
        if self.rotation_accum >= (2 * math.pi):
            self.finish_buzzer_rotation()

    def finish_buzzer_rotation(self):
        if self.buzzer_timer:
            self.buzzer_timer.cancel()
            self.buzzer_timer = None

        stop = Twist()
        self.manual_pub.publish(stop)

        self.set_mode("IDLE")
        self.get_logger().info("★★ 부저 + 한바퀴 회전 완료 ★★")

    def call_buzzer_sound(self, value: int):
        if not self.sound_client.wait_for_service(timeout_sec=0.05):
            return
        req = Sound.Request()
        req.value = value
        self.sound_client.call_async(req)

    # ============================================================
    # goal_reached 콜백 (정선/난선 순찰 처리 핵심)
    # ============================================================
    def goal_reached_cb(self, msg: Bool):
        if not msg.data:
            return

        self.get_logger().info(f"[BM] Goal Reached, mode={self.current_mode}")

        # 1) 정선순찰: QR1→2→3→4→1→...
        if self.current_mode == "SEQ_PATROL":
            nxt = self.current_qr + 1
            if nxt > 4:
                nxt = 1
            self.current_qr = nxt
            self.send_goal_to_qr(nxt)
            return

        # 2) 난선순찰: 매번 랜덤 QR 선택
        if self.current_mode == "RANDOM_PATROL":
            nxt = random.randint(1, 4)
            self.current_qr = nxt
            self.send_goal_to_qr(nxt)
            return

        # 3) 복귀 완료, GOTO_QR 완료 등 나머지는 IDLE로
        self.set_mode("IDLE")

    # ============================================================
    # 교대(향후 2대 로봇용) – 지금은 크게 쓰진 않음
    # ============================================================
    def start_handover(self):
        if self.last_scanned_qr == 0:
            self.last_scanned_qr = self.current_qr

        msg = Int32()
        msg.data = self.last_scanned_qr
        self.handover_pub.publish(msg)
        self.get_logger().info(f"[HANDOVER] publish last_scanned_qr={self.last_scanned_qr}")

    def handover_cb(self, msg: Int32):
        # 1대만 쓸 땐 크게 의미 없음. 나중에 확장용.
        self.get_logger().info(f"[HANDOVER] received last_qr={msg.data}")

    # ============================================================
    # 장애물 감지 이벤트 (필요시 외부에서 alarm_event 발행)
    # ============================================================
    def alarm_cb(self, msg: Bool):
        if msg.data:
            self.get_logger().warn("Moving obstacle detected -> BUZZER behavior")
            self.start_buzzer_rotation()

    # ============================================================
    # E-STOP
    # ============================================================
    def emergency_stop(self):
        # 내비게이션 취소 + 속도 0
        self.send_nav_cancel()
        stop = Twist()
        self.manual_pub.publish(stop)

        # 부저 타이머도 중단
        if self.buzzer_timer:
            self.buzzer_timer.cancel()
            self.buzzer_timer = None

        self.set_mode("E_STOP")
        self.get_logger().warn("Emergency Stop!")


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
