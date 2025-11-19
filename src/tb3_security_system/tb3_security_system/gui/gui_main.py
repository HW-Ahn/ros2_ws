#!/usr/bin/env python3
import sys
import threading
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist

from PyQt5 import QtWidgets, QtCore, uic

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from PyQt5.QtGui import QImage, QPixmap

import numpy as np
import cv2


# ============================================================
#   GUI <-> ROS 브릿지 노드
# ============================================================
class ControlBridge(Node):
    """
    GUI <-> ROS 브리지 노드.
    - /tb3_1/high_level_cmd, /tb3_2/high_level_cmd (String)
    - /tb3_1/cmd_goal, /tb3_2/cmd_goal (PoseStamped)
    - /tb3_1/cmd_vel_manual, /tb3_2/cmd_vel_manual (Twist)
    - /tb3_1/state, /tb3_2/state (String)
    - 카메라: raw/compressed 선택적으로 구독
    """

    def __init__(self):
        super().__init__("tb3_gui_bridge")

        # ---------- 파라미터 ----------
        self.declare_parameter("use_compressed", False)
        self.declare_parameter("robot1_image_topic", "/tb3_1/camera/image_raw")
        self.declare_parameter("robot2_image_topic", "/tb3_2/camera/image_raw")

        self.use_compressed: bool = self.get_parameter("use_compressed").value
        self.image_topic_1: str = self.get_parameter("robot1_image_topic").value
        self.image_topic_2: str = self.get_parameter("robot2_image_topic").value

        self.get_logger().info(f"[GUI] use_compressed = {self.use_compressed}")
        self.get_logger().info(f"[GUI] Robot1 Image Topic = {self.image_topic_1}")
        self.get_logger().info(f"[GUI] Robot2 Image Topic = {self.image_topic_2}")

        # ---------- 퍼블리셔 ----------
        self.cmd_pub = {
            "tb3_1": self.create_publisher(String, "/tb3_1/high_level_cmd", 10),
            "tb3_2": self.create_publisher(String, "/tb3_2/high_level_cmd", 10),
        }
        self.goal_pub = {
            "tb3_1": self.create_publisher(PoseStamped, "/tb3_1/cmd_goal", 10),
            "tb3_2": self.create_publisher(PoseStamped, "/tb3_2/cmd_goal", 10),
        }
        self.vel_pub = {
            "tb3_1": self.create_publisher(Twist, "/tb3_1/cmd_vel_manual", 10),
            "tb3_2": self.create_publisher(Twist, "/tb3_2/cmd_vel_manual", 10),
        }

        # ---------- 상태 ----------
        self.robot_state = {
            "tb3_1": "IDLE",
            "tb3_2": "IDLE",
        }

        # GUI(MainWindow) 객체 참조
        self.window = None

        # cv_bridge
        self.cv_bridge = CvBridge()

        # ---------- 상태 구독 ----------
        self.create_subscription(
            String,
            "/tb3_1/state",
            lambda msg: self.state_cb("tb3_1", msg),
            10,
        )
        self.create_subscription(
            String,
            "/tb3_2/state",
            lambda msg: self.state_cb("tb3_2", msg),
            10,
        )

        # ---------- 카메라 구독 생성 (raw / compressed) ----------
        self.create_image_subscribers()

        # ---------- QR 포인트 좌표 ----------
        # 1~4번 박스 앞 좌표 (필요시 조정)
        self.qr_points = {
            1: (-1.8, -2.2, 0.0),
            2: (1.8, -2.2, math.pi),
            3: (1.8, 2.2, math.pi / 2),
            4: (-1.8, 2.2, -math.pi / 2),
        }

        self.get_logger().info("ControlBridge initialized")

    # ============================================================
    #   GUI(MainWindow) 연결
    # ============================================================
    def set_window(self, window):
        self.window = window

    # ============================================================
    #   카메라 구독 (raw / compressed)
    # ============================================================
    def create_image_subscribers(self):
        """use_compressed 플래그와 토픽 파라미터에 따라 구독 설정"""

        if self.use_compressed:
            # launch에서 넘어온 토픽 그대로 사용
            topic1 = self.image_topic_1
            topic2 = self.image_topic_2
            self.get_logger().info(
                f"[GUI] Subscribing COMPRESSED images:\n  {topic1}\n  {topic2}"
            )

            self.img_sub_1 = self.create_subscription(
                CompressedImage,
                topic1,
                lambda msg: self.image_cb_compressed("tb3_1", msg),
                10,
            )
            self.img_sub_2 = self.create_subscription(
                CompressedImage,
                topic2,
                lambda msg: self.image_cb_compressed("tb3_2", msg),
                10,
            )
        else:
            topic1 = self.image_topic_1
            topic2 = self.image_topic_2
            self.get_logger().info(
                f"[GUI] Subscribing RAW images:\n  {topic1}\n  {topic2}"
            )

            self.img_sub_1 = self.create_subscription(
                Image,
                topic1,
                lambda msg: self.image_cb_raw("tb3_1", msg),
                10,
            )
            self.img_sub_2 = self.create_subscription(
                Image,
                topic2,
                lambda msg: self.image_cb_raw("tb3_2", msg),
                10,
            )

    # ============================================================
    #   상태 콜백
    # ============================================================
    def state_cb(self, robot: str, msg: String):
        self.robot_state[robot] = msg.data
        # 상태를 GUI에 바로 반영하고 싶으면 여기서 window에 알려줄 수도 있음

    # ============================================================
    #   이미지 콜백 (RAW)
    # ============================================================
    def image_cb_raw(self, robot: str, msg: Image):
        if self.window is None:
            return

        active_robot = getattr(self.window, "active_robot", None)
        if active_robot != robot:
            # 선택된 로봇이 아니면 표시하지 않음
            return

        try:
            cv_img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"[RAW] cv_bridge 변환 실패: {e}")
            return

        self.publish_to_gui(robot, cv_img)

    # ============================================================
    #   이미지 콜백 (COMPRESSED)
    # ============================================================
    def image_cb_compressed(self, robot: str, msg: CompressedImage):
        if self.window is None:
            return

        active_robot = getattr(self.window, "active_robot", None)
        if active_robot != robot:
            return

        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().warn(f"[COMPRESSED] 디코딩 실패: {e}")
            return

        if cv_img is None:
            self.get_logger().warn("[COMPRESSED] 디코딩 결과가 None")
            return

        self.publish_to_gui(robot, cv_img)

    # ============================================================
    #   QImage로 변환 후 GUI에 전달 (공통)
    # ============================================================
    def publish_to_gui(self, robot: str, cv_img):
        h, w, ch = cv_img.shape
        bytes_per_line = ch * w
        qt_img = QImage(
            cv_img.data, w, h, bytes_per_line, QImage.Format.Format_BGR888
        )
        self.window.update_camera(robot, qt_img)

    # ============================================================
    #   고수준 명령 / 수동 속도 / QR 이동
    # ============================================================
    def send_high_level_cmd(self, robot: str, cmd: str):
        if robot not in self.cmd_pub:
            self.get_logger().error(f"Unknown robot {robot}")
            return
        msg = String()
        msg.data = cmd
        self.cmd_pub[robot].publish(msg)
        self.get_logger().info(f"[{robot}] high_level_cmd: {cmd}")

    def send_goal_to_qr(self, robot: str, qr_index: int):
        """QR 번호에 해당하는 좌표로 goal 송신 (사용 여부는 선택)"""
        if robot not in self.goal_pub:
            self.get_logger().error(f"Unknown robot {robot}")
            return
        if qr_index not in self.qr_points:
            self.get_logger().error(f"Unknown QR index {qr_index}")
            return

        x, y, yaw = self.qr_points[qr_index]
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = "map"
        ps.pose.position.x = x
        ps.pose.position.y = y
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw

        self.goal_pub[robot].publish(ps)
        self.get_logger().info(
            f"[{robot}] send_goal_to_qr{qr_index}: ({x:.2f},{y:.2f},{yaw:.2f})"
        )

    def send_manual_vel(self, robot: str, vx: float, vy: float, wz: float):
        if robot not in self.vel_pub:
            self.get_logger().error(f"Unknown robot {robot}")
            return
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        self.vel_pub[robot].publish(msg)


# ============================================================
#   MainWindow (PyQt5)
# ============================================================
class MainWindow(QtWidgets.QDialog):
    def __init__(self, bridge: ControlBridge):
        super().__init__()
        # UI 파일 로드
        uic.loadUi(
            "/home/polestar3/ros2_ws/src/tb3_security_system/ui/tb3_control.ui",
            self,
        )

        self.bridge = bridge
        self.active_robot = "tb3_1"  # 기본 선택 로봇
        self.bridge.set_window(self)

        # ---------------------- 스케일링용 초기값 저장 ----------------------
        self._base_size = self.size()
        self._orig_geometries = {}
        for w in self.findChildren(QtWidgets.QWidget):
            if w is self:
                continue
            self._orig_geometries[w] = w.geometry()
        # ------------------------------------------------------------------

        # 상단에 현재 로봇 표시용 라벨
        if hasattr(self, "label_active"):
            self.label_active.setText(self.active_robot)

        # 방향 키용 타이머
        self.manual_timer = QtCore.QTimer(self)
        self.manual_timer.setInterval(100)  # 10Hz
        self.manual_timer.timeout.connect(self.on_manual_timer)
        self.manual_cmd = (0.0, 0.0, 0.0)  # vx, vy, wz

        # 버튼 연결
        self.btn_switch.clicked.connect(self.on_switch_robot)

        self.update_active_label()

        # QR 이동
        self.btn_qr1.clicked.connect(lambda: self.on_qr_button(1))
        self.btn_qr2.clicked.connect(lambda: self.on_qr_button(2))
        self.btn_qr3.clicked.connect(lambda: self.on_qr_button(3))
        self.btn_qr4.clicked.connect(lambda: self.on_qr_button(4))

        # QR 스캔
        self.btn_qrscan.clicked.connect(self.on_qr_scan)

        # 부저
        self.btn_buzzer.clicked.connect(self.on_buzzer)

        # 정선순찰 (btn_start 재활용)
        self.btn_start.clicked.connect(self.on_seq_patrol)

        # 난선순찰/복귀 버튼 (UI에 있을 경우만)
        if hasattr(self, "btn_patrol_random"):
            self.btn_patrol_random.clicked.connect(self.on_random_patrol)
        if hasattr(self, "btn_return"):
            self.btn_return.clicked.connect(self.on_return_home)

        # 비상정지, 종료
        self.btn_estop.clicked.connect(self.on_estop)
        self.btn_stop.clicked.connect(self.close)

        # 방향 버튼: pressed / released 사용
        self.btn_up.pressed.connect(
            lambda: self.set_manual_cmd(0.1, 0.0, 0.0)
        )
        self.btn_down.pressed.connect(
            lambda: self.set_manual_cmd(-0.1, 0.0, 0.0)
        )
        self.btn_left.pressed.connect(
            lambda: self.set_manual_cmd(0.0, 0.0, 0.5)
        )
        self.btn_right.pressed.connect(
            lambda: self.set_manual_cmd(0.0, 0.0, -0.5)
        )

        self.btn_up.released.connect(self.stop_manual_cmd)
        self.btn_down.released.connect(self.stop_manual_cmd)
        self.btn_left.released.connect(self.stop_manual_cmd)
        self.btn_right.released.connect(self.stop_manual_cmd)

        # 카메라 레이블 (원본 비율 유지)
        self.label_cam1.setScaledContents(True)
        self.label_cam2.setScaledContents(True)

    # ---------------------- 창 리사이즈 시 전체 스케일링 ----------------------
    def resizeEvent(self, event):
        super().resizeEvent(event)
        if not self._base_size.width() or not self._base_size.height():
            return

        sx = self.width() / self._base_size.width()
        sy = self.height() / self._base_size.height()
        s = min(sx, sy)

        for w, g in self._orig_geometries.items():
            new_rect = QtCore.QRect(
                int(g.x() * s),
                int(g.y() * s),
                int(g.width() * s),
                int(g.height() * s),
            )
            w.setGeometry(new_rect)
    # ----------------------------------------------------------------------

    # ------------------------
    # 내부 유틸
    # ------------------------
    def update_active_label(self):
        if hasattr(self, "label_active"):
            self.label_active.setText(f"현재 로봇: {self.active_robot}")

    def log(self, msg: str):
        """간단 로그 출력 (선택된 로봇에 따라 text_log1/2에 출력)"""
        if self.active_robot == "tb3_1" and hasattr(self, "text_log1"):
            self.text_log1.append(msg)
        elif self.active_robot == "tb3_2" and hasattr(self, "text_log2"):
            self.text_log2.append(msg)

    # ------------------------
    # 버튼 콜백
    # ------------------------
    def on_switch_robot(self):
        self.active_robot = "tb3_2" if self.active_robot == "tb3_1" else "tb3_1"
        self.update_active_label()
        self.log(f"제어 대상 전환: {self.active_robot}")

    def on_qr_button(self, idx: int):
        cmd = f"GOTO_QR{idx}"
        self.bridge.send_high_level_cmd(self.active_robot, cmd)
        self.log(f"{self.active_robot} → {idx}번 포인트로 이동 명령 ({cmd})")

    def on_qr_scan(self):
        self.bridge.send_high_level_cmd(self.active_robot, "QR_SCAN_NEXT")
        self.log(f"{self.active_robot} → QR 스캔 + 다음 포인트 이동 명령")

    def on_buzzer(self):
        self.bridge.send_high_level_cmd(self.active_robot, "BUZZER")
        self.log(f"{self.active_robot} → 부저 동작 명령")

    def on_seq_patrol(self):
        self.bridge.send_high_level_cmd(self.active_robot, "SEQ_PATROL_START")
        self.log(f"{self.active_robot} → 정선순찰 시작")

    def on_random_patrol(self):
        self.bridge.send_high_level_cmd(self.active_robot, "RANDOM_PATROL_START")
        self.log(f"{self.active_robot} → 난선순찰 시작")

    def on_return_home(self):
        self.bridge.send_high_level_cmd(self.active_robot, "RETURN_HOME")
        self.log(f"{self.active_robot} → 복귀 명령")

    def on_estop(self):
        # 두 로봇 모두 비상정지
        self.bridge.send_high_level_cmd("tb3_1", "EMERGENCY_STOP")
        self.bridge.send_high_level_cmd("tb3_2", "EMERGENCY_STOP")
        self.log("비상정지 명령 전송 (두 로봇 모두)")

    # ------------------------
    # 수동 조종 관련
    # ------------------------
    def set_manual_cmd(self, vx, vy, wz):
        self.manual_cmd = (vx, vy, wz)
        if not self.manual_timer.isActive():
            self.manual_timer.start()

    def stop_manual_cmd(self):
        self.manual_cmd = (0.0, 0.0, 0.0)
        self.manual_timer.stop()
        self.bridge.send_manual_vel(self.active_robot, 0.0, 0.0, 0.0)

    def on_manual_timer(self):
        vx, vy, wz = self.manual_cmd
        self.bridge.send_manual_vel(self.active_robot, vx, vy, wz)

    @QtCore.pyqtSlot(str, QImage)
    def update_camera(self, robot: str, img: QImage):
        """
        ControlBridge에서 호출하는 슬롯.
        선택된 로봇의 이미지만 갱신.
        """
        pix = QPixmap.fromImage(img)

        if robot == "tb3_1":
            if hasattr(self, "label_cam1"):
                self.label_cam1.setPixmap(pix)
        elif robot == "tb3_2":
            if hasattr(self, "label_cam2"):
                self.label_cam2.setPixmap(pix)


# ============================================================
#   rclpy 스레드 + Qt 실행
# ============================================================
def ros_spin_thread(executor):
    executor.spin()


def main():
    rclpy.init()
    bridge = ControlBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(bridge)

    # ROS2 스핀을 백그라운드 스레드로
    t = threading.Thread(target=ros_spin_thread, args=(executor,), daemon=True)
    t.start()

    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow(bridge)
    win.show()
    ret = app.exec_()

    executor.shutdown()
    bridge.destroy_node()
    rclpy.shutdown()
    sys.exit(ret)


if __name__ == "__main__":
    main()
