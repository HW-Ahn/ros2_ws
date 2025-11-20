# 파일: tb3_security_system/nodes/behavior_manager.py

import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import String, Bool, Int32
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from turtlebot3_msgs.srv import Sound


def yaw_to_quat(yaw: float):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class BehaviorManager(Node):
    def __init__(self):
        super().__init__("behavior_manager")

        # 네임스페이스
        ns = self.get_namespace().stㅁrip('/')
        self.robot_ns = ns if ns else "tb3_1"
        self.is_primary = (self.robot_ns == "tb3_1")

        # 상태 변수
        self.current_mode = "IDLE"
        self.current_qr = 1
        self.last_scanned_qr = 0
        self.home_pose = None  # 최초 odom 좌표 자동 저장

        # ---------- QR 위치 (수정된 좌표 포함) ----------
        self.qr_scan_poses = {
            1: (1.2, -1.5, -math.pi/2),   # 수정된 QR1
            2: (4.7, -1.15, -math.pi/2),
            3: (4.4,  2.2,  math.pi/2),
            4: (0.6, 2.2,   math.pi/2),   # 수정된 QR4
        }

        # ---------- Publisher / Subscriber ----------
        self.state_pub = self.create_publisher(String, 'state', 10)
        self.goal_pub = self.create_publisher(PoseStamped, 'cmd_goal', 10)
        self.manual_pub = self.create_publisher(Twist, 'cmd_vel_manual', 10)
        self.nav_cancel_pub = self.create_publisher(Bool, 'nav_cancel', 10)

        self.cmd_sub = self.create_subscription(String, 'high_level_cmd', self.cmd_cb, 10)
        self.goal_reached_sub = self.create_subscription(Bool, 'goal_reached', self.goal_reached_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)

        # ---------- 부저 서비스 클라이언트 ----------
        self.sound_client = self.create_client(Sound, 'sound')

        # ---------- 부저 반복 + 회전용 변수 ----------
        self.buzzer_timer = None
        self.rotation_accum = 0.0
        self.last_yaw = None

        self.set_mode("IDLE")
        self.get_logger().info(f"BehaviorManager started for {self.robot_ns}")

    def send_nav_cancel(self):
        msg = Bool()
        msg.data = True
        self.nav_cancel_pub.publish(msg)

    # ============================================================
    # Home Pose 저장 (초기 odom 자동 저장)
    # ============================================================
    def odom_cb(self, msg: Odometry):
        if self.home_pose is None:
            self.home_pose = msg.pose.pose
            self.get_logger().info("*** Home Pose (odom) 저장 완료 ***")
        # yaw 추적용
        ori = msg.pose.pose.orientation
        _, _, yaw = self.quat_to_yaw(ori.x, ori.y, ori.z, ori.w)
        if self.last_yaw is None:
            self.last_yaw = yaw
        self.current_yaw = yaw

    def quat_to_yaw(self, x, y, z, w):
        siny = 2.0 * (w*z + x*y)
        cosy = 1.0 - 2.0 * (y*y + z*z)
        return (0.0, 0.0, math.atan2(siny, cosy))

    # ============================================================
    # 상태 관리
    # ============================================================
    def set_mode(self, mode: str):
        self.current_mode = mode
        msg = String()
        msg.data = mode
        self.state_pub.publish(msg)

    # ============================================================
    # 상위 명령 수신
    # ============================================================
    def cmd_cb(self, msg: String):
        cmd = msg.data
        self.get_logger().info(f"CMD: {cmd}")

        if cmd.startswith("GOTO_QR"):
            idx = int(cmd[-1])
            self.current_qr = idx
            self.send_goal_to_qr(idx)
            self.set_mode("GOTO_QR")

        elif cmd == "RETURN_HOME":
            self.handle_return_home()

        elif cmd == "BUZZER":
            self.start_buzzer_rotation()

        elif cmd == "SEQ_PATROL_START":
            self.set_mode("SEQ_PATROL")
            self.current_qr = 1
            self.send_goal_to_qr(1)

        elif cmd == "EMERGENCY_STOP":
            self.emergency_stop()

    # ============================================================
    # QR 이동
    # ============================================================
    def send_goal_to_qr(self, idx):
        if idx not in self.qr_scan_poses:
            self.get_logger().error("QR index error")
            return

        x, y, yaw = self.qr_scan_poses[idx]

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "odom"     # A안 유지

        pose.pose.position.x = x
        pose.pose.position.y = y

        qx, qy, qz, qw = yaw_to_quat(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.goal_pub.publish(pose)
        self.get_logger().info(f"[BM] 이동 목표 QR{idx}: ({x:.2f}, {y:.2f}) yaw={yaw:.2f}")

    # ============================================================
    # RETURN_HOME
    # ============================================================
    def handle_return_home(self):
        if self.home_pose is None:
            self.get_logger().warn("★ home_pose가 저장되지 않았습니다.")
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
        self.buzzer_timer = self.create_timer(0.1, self.buzzer_loop)

    def buzzer_loop(self):
        # 각속도 유지
        twist = Twist()
        twist.angular.z = -1.0
        self.manual_pub.publish(twist)

        # 사운드 호출
        self.call_buzzer_sound(3)

        # 회전량 누적
        try:
            dyaw = abs(self.current_yaw - self.last_yaw)
            if dyaw > math.pi:
                dyaw = (2*math.pi) - dyaw
            self.rotation_accum += dyaw
        except:
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

    # ------------------------------------------------------------
    def call_buzzer_sound(self, value: int):
        if not self.sound_client.wait_for_service(timeout_sec=0.1):
            return
        req = Sound.Request()
        req.value = value
        self.sound_client.call_async(req)

    # ============================================================
    # 목표 도착 콜백
    # ============================================================
    def goal_reached_cb(self, msg: Bool):
        if not msg.data:
            return
        self.get_logger().info(f"[BM] Goal Reached, mode={self.current_mode}")
        self.set_mode("IDLE")

    # ============================================================
    # E-STOP
    # ============================================================
    def emergency_stop(self):
        self.send_nav_cancel()
        stop = Twist()
        self.manual_pub.publish(stop)
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
