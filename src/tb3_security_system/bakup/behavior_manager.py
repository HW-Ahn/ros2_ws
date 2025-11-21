# 파일: tb3_security_system/nodes/behavior_manager.py

import math
import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int32
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from turtlebot3_msgs.srv import Sound


def yaw_to_quat(yaw: float):
    """2D yaw -> 쿼터니언 (x,y,z,w)"""
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class BehaviorManager(Node):
    def __init__(self):
        super().__init__("behavior_manager")

        # 네임스페이스 (/tb3_1, /tb3_2 ...)
        ns = self.get_namespace().strip('/')
        self.robot_ns = ns if ns else "tb3_1"
        self.is_primary = (self.robot_ns == "tb3_1")

        # 상태 변수
        self.current_mode = "IDLE"
        self.current_qr = 1
        self.last_scanned_qr = 0

        # home_pose: 최초 odom을 자동으로 저장
        self.home_pose = None

        # ---------- QR 목표 위치 (odom 기준) ----------
        self.qr_scan_poses = {
            1: (1.0, -1.7, -math.pi / 2.0),
            2: (4.5, -1.5, -math.pi / 2.0),
            3: (4.4,  2.0,  math.pi / 2.0),
            4: (1.0,  2.0,  math.pi / 2.0),
        }

        # ---------- 움직이는 장애물 모니터링 파라미터 ----------
        # 정면 기준 감지 각도 (±deg)
        self.declare_parameter('monitor_angle_deg', 60.0)
        # 이 거리 안쪽 레이만 관심 대상으로 사용
        self.declare_parameter('monitor_distance', 1.0)
        # 로봇 이동으로 설명 가능한 거리 변화 + 여유 마진
        self.declare_parameter('monitor_delta_margin', 0.07)
        # 한 프레임에서 "이상하게 많이 변한" 레이 개수가 이 값 이상이면 동적 후보
        self.declare_parameter('monitor_changed_beams', 2)
        # 그런 프레임이 연속으로 몇 번 나와야 부저 울릴지
        self.declare_parameter('monitor_hit_count', 2)
        # 동적 감지를 수행할 최소 선속도 / 최대 각속도
        self.declare_parameter('dyn_min_linear_speed', 0.05)
        self.declare_parameter('dyn_max_angular_speed', 0.8)

        self.monitor_angle = math.radians(
            float(self.get_parameter('monitor_angle_deg').value)
        )
        self.monitor_distance = float(self.get_parameter('monitor_distance').value)
        self.monitor_delta_margin = float(self.get_parameter('monitor_delta_margin').value)
        self.monitor_changed_beams = int(self.get_parameter('monitor_changed_beams').value)
        self.monitor_hit_needed = int(self.get_parameter('monitor_hit_count').value)

        self.dyn_min_v = float(self.get_parameter('dyn_min_linear_speed').value)
        self.dyn_max_w = float(self.get_parameter('dyn_max_angular_speed').value)

        # 모니터링용 상태
        self.prev_front_ranges = None
        self.prev_scan_stamp = None
        self.monitor_hit_count = 0

        # 로봇 속도 (odom에서 갱신)
        self.robot_v = 0.0  # 선속도
        self.robot_w = 0.0  # 각속도

        # ---------- 퍼블리셔 / 서브스크라이버 ----------
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

        # 라이다 스캔 (움직이는 장애물 감지용)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_cb, 10
        )

        # ---------- 부저 서비스 ----------
        self.sound_client = self.create_client(Sound, 'sound')

        # ---------- 부저 반복 + 회전용 ----------
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

    @staticmethod
    def normalize_angle(a: float) -> float:
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    # ============================================================
    # odom 콜백 (home_pose 저장 + 속도 추적)
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

        # 속도 정보 저장
        self.robot_v = msg.twist.twist.linear.x
        self.robot_w = msg.twist.twist.angular.z

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
            self.current_qr = 1
            self.set_mode("SEQ_PATROL")
            self.send_goal_to_qr(1)

        elif cmd == "RANDOM_PATROL_START":
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
            self.start_handover()

        elif cmd == "QR_SCAN_NEXT":
            # 실제 QR 스캔과 상관 없이 "다음 QR로 이동" 기능만 구현
            nxt = self.current_qr + 1
            if nxt > 4:
                nxt = 1
            self.current_qr = nxt
            self.send_goal_to_qr(nxt)
            self.set_mode("GOTO_QR")

        else:
            self.get_logger().warn(f"Unknown CMD: {cmd}")

    # ============================================================
    # QR 이동 목표 전송
    # ============================================================
    def send_goal_to_qr(self, idx: int):
        if idx not in self.qr_scan_poses:
            self.get_logger().error(f"QR index {idx} not defined")
            return

        x, y, yaw = self.qr_scan_poses[idx]

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "odom"

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
    # 라이다 기반 움직이는 장애물 감지
    # ============================================================
    def scan_cb(self, msg: LaserScan):
        """
        - 정면 ± monitor_angle 범위 안의 레이들만 사용
        - 이전 스캔과 비교했을 때,
          1) 로봇 이동(v*dt + margin)으로 설명하기 힘든 큰 변화
          2) inf → 유한, 유한 → inf 변화
          가 여러 레이에서 동시에 발생하면 → 움직이는 장애물로 판단
        - 제자리 회전에 가까운 구간(|w| 큰 경우)은 감지하지 않음
        """

        # E-STOP / BUZZER 중에는 감지 OFF
        if self.current_mode in ["E_STOP", "BUZZER"]:
            self.monitor_hit_count = 0
            self.prev_front_ranges = None
            self.prev_scan_stamp = None
            return

        # 제자리 회전에 가까우면 감지 끔 (정지 장애물도 패턴이 크게 바뀜)
        if abs(self.robot_w) > self.dyn_max_w:
            self.monitor_hit_count = 0
            self.prev_front_ranges = None
            self.prev_scan_stamp = msg.header.stamp
            return

        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        ranges = msg.ranges

        # 시간 간격 dt 계산
        if self.prev_scan_stamp is None:
            dt = None
        else:
            now = msg.header.stamp
            dt = (now.sec - self.prev_scan_stamp.sec) + \
                 (now.nanosec - self.prev_scan_stamp.nanosec) * 1e-9
        self.prev_scan_stamp = msg.header.stamp

        # 정면 섹터 레이 추출
        front_ranges = []
        for i, r in enumerate(ranges):
            angle = angle_min + i * angle_inc
            rel = self.normalize_angle(angle)  # 라이다 기준 정면이 0rad 라고 가정

            if not (-self.monitor_angle <= rel <= self.monitor_angle):
                continue

            if (
                r == 0.0
                or math.isinf(r)
                or math.isnan(r)
                or r < msg.range_min
                or r > msg.range_max
            ):
                front_ranges.append(float('inf'))
            else:
                front_ranges.append(r)

        if len(front_ranges) == 0:
            self.prev_front_ranges = None
            return

        # 첫 프레임이면 기준만 저장
        if self.prev_front_ranges is None or dt is None or dt <= 0.0:
            self.prev_front_ranges = front_ranges
            self.monitor_hit_count = 0
            return

        v = abs(self.robot_v)
        expected_max_delta = v * dt + self.monitor_delta_margin

        dynamic_beams = 0
        min_front = float('inf')

        for prev_r, curr_r in zip(self.prev_front_ranges, front_ranges):

            # 1) 둘 다 INF이면 비교 불가 → 패스
            if prev_r == float('inf') and curr_r == float('inf'):
                continue

            # 2) 이전에는 없었는데 (INF) 지금은 가까이 생김 → "갑자기 나타난 물체"
            if prev_r == float('inf') and curr_r < self.monitor_distance:
                dynamic_beams += 1
                if curr_r < min_front:
                    min_front = curr_r
                continue

            # 3) 이전에는 가까이 있었는데 지금은 사라짐 → "갑자기 사라진 물체"
            if curr_r == float('inf') and prev_r < self.monitor_distance:
                dynamic_beams += 1
                if prev_r < min_front:
                    min_front = prev_r
                continue

            # 4) 둘 다 유한값일 때: 너무 멀면 관심 없음
            if prev_r > self.monitor_distance and curr_r > self.monitor_distance:
                continue

            delta = abs(curr_r - prev_r)

            if curr_r < min_front:
                min_front = curr_r

            # 로봇 이동만으로 설명하기 어려운 큰 변화라면 동적 후보
            if delta > expected_max_delta:
                dynamic_beams += 1

        # 다음 비교를 위해 저장
        self.prev_front_ranges = front_ranges

        self.get_logger().info(
            f"[BM] dyn_monitor: dyn_beams={dynamic_beams}, "
            f"hit={self.monitor_hit_count}, v={self.robot_v:.2f}, "
            f"w={self.robot_w:.2f}, dt={dt:.3f}, "
            f"min_front={min_front if min_front < float('inf') else -1:.2f}"
        )

        # 부저 발동 후보 조건
        moving_candidate = (
            dynamic_beams >= self.monitor_changed_beams and
            min_front < self.monitor_distance
            # v 조건은 일부러 빼서, 로봇이 거의 멈춰 있어도 감지되도록 함
        )

        if moving_candidate:
            self.monitor_hit_count += 1
        else:
            if self.monitor_hit_count > 0:
                self.monitor_hit_count -= 1

        if self.monitor_hit_count >= self.monitor_hit_needed:
            self.get_logger().warn(
                f"[BM] 움직이는 장애물 감지: dyn_beams={dynamic_beams}, "
                f"min_front={min_front if min_front < float('inf') else -1:.2f} → BUZZER"
            )
            self.monitor_hit_count = 0
            self.start_buzzer_rotation()

    # ============================================================
    # 부저 + 한 바퀴 회전
    # ============================================================
    def start_buzzer_rotation(self):
        self.send_nav_cancel()

        self.rotation_accum = 0.0
        self.last_yaw = None
        self.set_mode("BUZZER")

        if self.buzzer_timer is not None:
            self.buzzer_timer.cancel()
        self.buzzer_timer = self.create_timer(0.1, self.buzzer_loop)
        self.get_logger().info("[BM] BUZZER 회전 시작")

    def buzzer_loop(self):
        twist = Twist()
        twist.angular.z = -1.0
        self.manual_pub.publish(twist)

        self.call_buzzer_sound(3)

        try:
            dyaw = abs(self.current_yaw - self.last_yaw)
            if dyaw > math.pi:
                dyaw = (2 * math.pi) - dyaw
            self.rotation_accum += dyaw
        except Exception:
            pass

        self.last_yaw = self.current_yaw

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
    # goal_reached 콜백
    # ============================================================
    def goal_reached_cb(self, msg: Bool):
        if not msg.data:
            return

        self.get_logger().info(f"[BM] Goal Reached, mode={self.current_mode}")

        if self.current_mode == "SEQ_PATROL":
            nxt = self.current_qr + 1
            if nxt > 4:
                nxt = 1
            self.current_qr = nxt
            self.send_goal_to_qr(nxt)
            return

        if self.current_mode == "RANDOM_PATROL":
            nxt = random.randint(1, 4)
            self.current_qr = nxt
            self.send_goal_to_qr(nxt)
            return

        self.set_mode("IDLE")

    # ============================================================
    # 교대 (향후 2대 로봇용)
    # ============================================================
    def start_handover(self):
        if self.last_scanned_qr == 0:
            self.last_scanned_qr = self.current_qr

        msg = Int32()
        msg.data = self.last_scanned_qr
        self.handover_pub.publish(msg)
        self.get_logger().info(f"[HANDOVER] publish last_scanned_qr={self.last_scanned_qr}")

    def handover_cb(self, msg: Int32):
        self.get_logger().info(f"[HANDOVER] received last_qr={msg.data}")

    # ============================================================
    # 외부 alarm_event 콜백
    # ============================================================
    def alarm_cb(self, msg: Bool):
        if msg.data:
            self.get_logger().warn(
                "Moving obstacle detected (external alarm_event) -> BUZZER behavior"
            )
            self.start_buzzer_rotation()

    # ============================================================
    # 비상 정지
    # ============================================================
    def emergency_stop(self):
        self.send_nav_cancel()
        stop = Twist()
        self.manual_pub.publish(stop)

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
