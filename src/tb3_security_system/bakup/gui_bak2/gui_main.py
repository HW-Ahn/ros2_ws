import sys
import threading
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist

from PyQt5 import QtWidgets, QtCore, uic
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QSizePolicy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ControlBridge(Node):
    """
    GUI <-> ROS 브리지 노드.
    - /tb3_1/high_level_cmd, /tb3_2/high_level_cmd (String)
    - /tb3_1/cmd_goal, /tb3_2/cmd_goal (PoseStamped)
    - /tb3_1/cmd_vel_manual, /tb3_2/cmd_vel_manual (Twist)
    - 카메라: /tb15/image_raw, /tb17/image_raw (Image)
      (v4l2_camera 노드를 __ns:=/tb15, /tb17 로 실행하는 설정 기준)
    """
    def __init__(self):
        super().__init__('tb3_gui_bridge')

        # 로봇별 퍼블리셔
        self.cmd_pub = {
            'tb3_1': self.create_publisher(String, '/tb3_1/high_level_cmd', 10),
            'tb3_2': self.create_publisher(String, '/tb3_2/high_level_cmd', 10),
        }
        self.goal_pub = {
            'tb3_1': self.create_publisher(PoseStamped, '/tb3_1/cmd_goal', 10),
            'tb3_2': self.create_publisher(PoseStamped, '/tb3_2/cmd_goal', 10),
        }
        self.vel_pub = {
            'tb3_1': self.create_publisher(Twist, '/tb3_1/cmd_vel_manual', 10),
            'tb3_2': self.create_publisher(Twist, '/tb3_2/cmd_vel_manual', 10),
        }

        # 상태 저장
        self.robot_state = {
            'tb3_1': 'IDLE',
            'tb3_2': 'IDLE',
        }

        # GUI 참조 (나중에 set_window로 주입)
        self.window = None

        # CvBridge
        self.cv_bridge = CvBridge()

        # 상태 구독 (있으면 사용)
        self.state_sub_1 = self.create_subscription(
            String, '/tb3_1/state',
            lambda msg: self.state_cb('tb3_1', msg),
            10
        )
        self.state_sub_2 = self.create_subscription(
            String, '/tb3_2/state',
            lambda msg: self.state_cb('tb3_2', msg),
            10
        )

        # ★ 카메라 구독: v4l2_camera 네임스페이스 /tb15, /tb17 기준
        self.img_sub_1 = self.create_subscription(
            Image, '/tb15/image_raw',
            lambda msg: self.image_cb('tb3_1', msg),
            10
        )
        self.img_sub_2 = self.create_subscription(
            Image, '/tb17/image_raw',
            lambda msg: self.image_cb('tb3_2', msg),
            10
        )

        # QR 포인트 예시 좌표
        self.qr_points = {
            1: (-1.8, -2.2, 0.0),
            2: (1.8, -2.2, math.pi),
            3: (1.8, 2.2, math.pi / 2),
            4: (-1.8, 2.2, -math.pi / 2),
        }

    # ---------- 유틸 ----------
    def set_window(self, window):
        self.window = window

    # ---------- 퍼블리시 함수 ----------
    def send_high_level_cmd(self, robot: str, cmd: str):
        if robot not in self.cmd_pub:
            self.get_logger().error(f'Unknown robot {robot}')
            return
        msg = String()
        msg.data = cmd
        self.cmd_pub[robot].publish(msg)
        self.get_logger().info(f'[{robot}] high_level_cmd: {cmd}')

    def send_goal_to_qr(self, robot: str, qr_index: int):
        if robot not in self.goal_pub or qr_index not in self.qr_points:
            return
        x, y, yaw = self.qr_points[qr_index]
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'map'
        ps.pose.position.x = x
        ps.pose.position.y = y
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        self.goal_pub[robot].publish(ps)
        self.get_logger().info(
            f'[{robot}] send_goal_to_qr{qr_index}: ({x:.2f},{y:.2f},{yaw:.2f})'
        )

    def send_manual_vel(self, robot: str, vx: float, vy: float, wz: float):
        if robot not in self.vel_pub:
            return
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        self.vel_pub[robot].publish(msg)

    # ---------- 콜백 ----------
    def state_cb(self, robot: str, msg: String):
        self.robot_state[robot] = msg.data

    def image_cb(self, robot: str, msg: Image):
        if self.window is None:
            return
        try:
            cv_img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge 변환 실패: {e}')
            return

        h, w, ch = cv_img.shape
        bytes_per_line = ch * w
        qimg = QImage(cv_img.data, w, h, bytes_per_line, QImage.Format_BGR888)

        # GUI에 전달
        self.window.update_camera(robot, qimg)


class MainWindow(QtWidgets.QDialog):
    def __init__(self, bridge: ControlBridge):
        super().__init__()

        # ★ gui_main.py 기준으로 .ui 경로 자동 계산
        base_dir = os.path.dirname(os.path.abspath(__file__))
        ui_path = '/home/polestar3/ros2_ws/src/tb3_security_system/ui/tb3_control.ui'
        # ui_path = os.path.normpath(
        #     os.path.join(base_dir, "..", "..", "ui", "tb3_control.ui")
        # )
        uic.loadUi(ui_path, self)

        self.bridge = bridge
        self.active_robot = 'tb3_1'  # 내부 로봇 이름
        self.setWindowTitle("TurtleBot3 Security GUI")

        # ===== 1. 전체 스케일링 기준 저장 =====
        self._base_size = self.size()
        self._orig_geometries = {}
        for w in self.findChildren(QtWidgets.QWidget):
            if w is self:
                continue
            self._orig_geometries[w] = w.geometry()

        # ===== 2. 카메라 프레임 / 라벨 세팅 =====
        self.frame_cam1 = getattr(self, "frame_cam1", None)
        self.frame_cam2 = getattr(self, "frame_cam2", None)

        # 만약 이름 안 줬으면 QFrame 두 개를 자동으로 픽
        if self.frame_cam1 is None or self.frame_cam2 is None:
            frames = [w for w in self.findChildren(QtWidgets.QFrame)]
            frames.sort(key=lambda f: f.objectName())
            if len(frames) >= 2:
                self.frame_cam1, self.frame_cam2 = frames[0], frames[1]

        # 카메라용 라벨: UI에 이미 있으면 그거 쓰고, 없으면 코드에서 생성
        self.cam_label1 = getattr(self, "label_cam1", None)
        self.cam_label2 = getattr(self, "label_cam2", None)

        if self.cam_label1 is None and self.frame_cam1 is not None:
            self.cam_label1 = QtWidgets.QLabel(self.frame_cam1)
            self.cam_label1.setObjectName("label_cam1")
            self.cam_label1.setAlignment(Qt.AlignCenter)

        if self.cam_label2 is None and self.frame_cam2 is not None:
            self.cam_label2 = QtWidgets.QLabel(self.frame_cam2)
            self.cam_label2.setObjectName("label_cam2")
            self.cam_label2.setAlignment(Qt.AlignCenter)

        if self.cam_label1 is not None:
            self.cam_label1.setScaledContents(True)
        if self.cam_label2 is not None:
            self.cam_label2.setScaledContents(True)

        # 프레임 크기에 맞춰 라벨 한 번 채우기
        self._update_cam_label_geometry()

        # ===== 3. 로그창 / 상태 라벨 =====
        self.text_log1 = getattr(self, "text_log1", None)
        self.text_log2 = getattr(self, "text_log2", None)

        # 상태 라벨 (파란색 / tb1, tb2 표시)
        self.label_active = getattr(self, "label_active", None)
        if self.label_active is not None:
            self.label_active.setObjectName("label_active")
            self.label_active.setStyleSheet("""
QLabel#label_active {
    color: #1890ff;
    font-weight: 700;
}
""")
            self.update_active_label()

        # ===== 4. 방향키용 타이머 =====
        self.manual_timer = QtCore.QTimer(self)
        self.manual_timer.setInterval(100)  # 10Hz
        self.manual_timer.timeout.connect(self.on_manual_timer)
        self.manual_cmd = (0.0, 0.0, 0.0)

        # ===== 5. 비상정지 btn_estop 스타일 + 연결 =====
        self.estop_button = None
        obj_name = None

        # 네 UI에서 사용하는 이름
        if hasattr(self, "btn_estop"):
            self.estop_button = self.btn_estop
            obj_name = "btn_estop"
        elif hasattr(self, "estop"):
            self.estop_button = self.estop
            obj_name = "estop"
        elif hasattr(self, "EMOButton"):
            self.estop_button = self.EMOButton
            obj_name = "EMOButton"

        if self.estop_button is not None and obj_name is not None:
            self.estop_button.setObjectName(obj_name)
            # 200 x 200 동그라미 + 레이아웃이 크기 안 늘리게 고정
            self.estop_button.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
            self.estop_button.setFixedSize(170, 170)

            self.estop_button.setStyleSheet(f"""
QPushButton#{obj_name} {{
    background:#ff4d4f;
    color:#000;
    border:8px solid #ffc400;
    border-radius:100px;   /* 200 / 2 -> 완전 동그라미 */
    font-weight:900;
    letter-spacing:1px;
    padding:0;
}}
QPushButton#{obj_name}:hover {{
    background:#f23b3d;
    border-color:#e6b800;
}}
QPushButton#{obj_name}:pressed {{
    background:#d9363e;
    border-color:#cca300;
    padding-top:1px;
    padding-left:1px;
}}""")
            self.estop_button.clicked.connect(self.on_estop)

        # ===== 6. 버튼 시그널 연결 =====

        # 로봇 전환
        if hasattr(self, "btn_switch"):
            self.btn_switch.clicked.connect(self.on_switch_robot)

        # QR 이동
        if hasattr(self, "btn_qr1"):
            self.btn_qr1.clicked.connect(lambda: self.on_qr_button(1))
        if hasattr(self, "btn_qr2"):
            self.btn_qr2.clicked.connect(lambda: self.on_qr_button(2))
        if hasattr(self, "btn_qr3"):
            self.btn_qr3.clicked.connect(lambda: self.on_qr_button(3))
        if hasattr(self, "btn_qr4"):
            self.btn_qr4.clicked.connect(lambda: self.on_qr_button(4))

        # QR 스캔
        if hasattr(self, "btn_qrscan"):
            self.btn_qrscan.clicked.connect(self.on_qr_scan)

        # 부저
        if hasattr(self, "btn_buzzer"):
            self.btn_buzzer.clicked.connect(self.on_buzzer)

        # 정선 순찰
        if hasattr(self, "btn_start"):
            self.btn_start.clicked.connect(self.on_seq_patrol)

        # 난선 순찰
        if hasattr(self, "btn_patrol_random"):
            self.btn_patrol_random.clicked.connect(self.on_random_patrol)

        # 복귀
        if hasattr(self, "btn_return"):
            self.btn_return.clicked.connect(self.on_return_home)

        # 종료 버튼
        if hasattr(self, "btn_stop"):
            self.btn_stop.clicked.connect(self.close)

        # 방향키
        if hasattr(self, "btn_up"):
            self.btn_up.pressed.connect(lambda: self.set_manual_cmd(0.1, 0.0, 0.0))
            self.btn_up.released.connect(self.stop_manual_cmd)
        if hasattr(self, "btn_down"):
            self.btn_down.pressed.connect(lambda: self.set_manual_cmd(-0.1, 0.0, 0.0))
            self.btn_down.released.connect(self.stop_manual_cmd)
        if hasattr(self, "btn_left"):
            self.btn_left.pressed.connect(lambda: self.set_manual_cmd(0.0, 0.0, 0.5))
            self.btn_left.released.connect(self.stop_manual_cmd)
        if hasattr(self, "btn_right"):
            self.btn_right.pressed.connect(lambda: self.set_manual_cmd(0.0, 0.0, -0.5))
            self.btn_right.released.connect(self.stop_manual_cmd)

    # ===== 상태 라벨 업데이트 =====
    def update_active_label(self):
        if self.label_active is None:
            return
        # 내부 이름 tb3_1 / tb3_2 를 사용자 표시 tb1 / tb2 로
        if self.active_robot == 'tb3_1':
            disp = 'tb1'
        elif self.active_robot == 'tb3_2':
            disp = 'tb2'
        else:
            disp = self.active_robot
        self.label_active.setText(f"현재 로봇: {disp}")

    # ===== 창 리사이즈 시 전체 스케일 + 카메라 라벨 맞추기 =====
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

        # 프레임 위치가 바뀌었으니, 라벨도 다시 프레임 크기에 맞춰줌
        self._update_cam_label_geometry()

    def _update_cam_label_geometry(self):
        if self.frame_cam1 is not None and self.cam_label1 is not None:
            self.cam_label1.setGeometry(self.frame_cam1.rect())
        if self.frame_cam2 is not None and self.cam_label2 is not None:
            self.cam_label2.setGeometry(self.frame_cam2.rect())

    # ===== 로그 유틸 =====
    def log_robot(self, robot: str, msg: str):
        line = f"[{robot}] {msg}"
        if robot == 'tb3_1' and self.text_log1 is not None:
            self.text_log1.append(line)
        elif robot == 'tb3_2' and self.text_log2 is not None:
            self.text_log2.append(line)
        else:
            print(line)

    def log_global(self, msg: str):
        if self.text_log1 is not None:
            self.text_log1.append(msg)
        if self.text_log2 is not None:
            self.text_log2.append(msg)
        if self.text_log1 is None and self.text_log2 is None:
            print(msg)

    # ===== 버튼 콜백 =====
    def on_switch_robot(self):
        self.active_robot = 'tb3_2' if self.active_robot == 'tb3_1' else 'tb3_1'
        self.update_active_label()
        self.log_robot(self.active_robot, "제어 대상 전환")

    def on_qr_button(self, idx: int):
        cmd = f"GOTO_QR{idx}"
        self.bridge.send_high_level_cmd(self.active_robot, cmd)
        self.log_robot(self.active_robot, f"{idx}번 QR 위치로 이동 명령 ({cmd})")

    def on_qr_scan(self):
        self.bridge.send_high_level_cmd(self.active_robot, "QR_SCAN_NEXT")
        self.log_robot(self.active_robot, "QR 스캔 후 다음 포인트 이동 명령")

    def on_buzzer(self):
        self.bridge.send_high_level_cmd(self.active_robot, "BUZZER")
        self.log_robot(self.active_robot, "부저 동작 명령")

    def on_seq_patrol(self):
        self.bridge.send_high_level_cmd(self.active_robot, "SEQ_PATROL_START")
        self.log_robot(self.active_robot, "정선 순찰 시작 명령")

    def on_random_patrol(self):
        self.bridge.send_high_level_cmd(self.active_robot, "RANDOM_PATROL_START")
        self.log_robot(self.active_robot, "난선 순찰 시작 명령")

    def on_return_home(self):
        self.bridge.send_high_level_cmd(self.active_robot, "RETURN_HOME")
        self.log_robot(self.active_robot, "복귀 명령")

    def on_estop(self):
        # 두 로봇 모두 비상 정지
        self.bridge.send_high_level_cmd('tb3_1', "EMERGENCY_STOP")
        self.bridge.send_high_level_cmd('tb3_2', "EMERGENCY_STOP")
        self.log_global("★ 비상정지 명령 전송 (tb3_1, tb3_2 모두)")

    # ===== 수동 조종 =====
    def set_manual_cmd(self, vx: float, vy: float, wz: float):
        self.manual_cmd = (vx, vy, wz)
        if not self.manual_timer.isActive():
            self.manual_timer.start()
        self.log_robot(self.active_robot, f"수동조종 시작 vx={vx:.2f}, wz={wz:.2f}")

    def stop_manual_cmd(self):
        self.manual_cmd = (0.0, 0.0, 0.0)
        if self.manual_timer.isActive():
            self.manual_timer.stop()
        self.bridge.send_manual_vel(self.active_robot, 0.0, 0.0, 0.0)
        self.log_robot(self.active_robot, "수동조종 정지")

    def on_manual_timer(self):
        vx, vy, wz = self.manual_cmd
        self.bridge.send_manual_vel(self.active_robot, vx, vy, wz)

    # ===== 카메라 업데이트 =====
    @QtCore.pyqtSlot(str, QImage)
    def update_camera(self, robot: str, img: QImage):
        pix = QPixmap.fromImage(img)

        if robot == 'tb3_1' and self.cam_label1 is not None:
            size = self.cam_label1.size()
            if size.width() > 0 and size.height() > 0:
                pix = pix.scaled(size, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.cam_label1.setPixmap(pix)

        elif robot == 'tb3_2' and self.cam_label2 is not None:
            size = self.cam_label2.size()
            if size.width() > 0 and size.height() > 0:
                pix = pix.scaled(size, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.cam_label2.setPixmap(pix)


# ===== rclpy 스레드 + Qt 실행 =====
def ros_spin_thread(executor):
    executor.spin()


def main():
    rclpy.init()
    bridge = ControlBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(bridge)

    t = threading.Thread(target=ros_spin_thread, args=(executor,), daemon=True)
    t.start()

    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow(bridge)
    bridge.set_window(win)
    win.show()
    ret = app.exec_()

    executor.shutdown()
    bridge.destroy_node()
    rclpy.shutdown()
    sys.exit(ret)


if __name__ == "__main__":
    main()
