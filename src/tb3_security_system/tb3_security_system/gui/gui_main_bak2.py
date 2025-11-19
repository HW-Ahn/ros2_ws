# 파일: ros2_ws/src/tb3_security_system/tb3_security_system/gui/gui_main.py
import sys
import threading
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist

from PyQt5 import QtWidgets, QtCore, uic

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from PyQt5.QtGui import QImage, QPixmap


class ControlBridge(Node):
    """
    GUI <-> ROS 브리지 노드.
    - /tb3_1/high_level_cmd, /tb3_2/high_level_cmd (String)
    - /tb3_1/cmd_goal, /tb3_2/cmd_goal (PoseStamped)
    - /tb3_1/cmd_vel_manual, /tb3_2/cmd_vel_manual (Twist)
    """
    def __init__(self):
        super().__init__('tb3_gui_bridge')

        # 로봇별 퍼블리셔 사전
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

        # 상태 저장용
        self.robot_state = {
            'tb3_1': 'IDLE',
            'tb3_2': 'IDLE',
        }

        # GUI(MainWindow) 객체 참조 (나중에 설정)
        self.window = None

        # cv_bridge
        self.cv_bridge = CvBridge()

        # 상태 구독
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

        # 카메라 이미지 구독 (토픽 이름은 실제 환경에 맞게 수정 필요)
        self.img_sub_1 = self.create_subscription(
            Image, '/tb3_1/camera/image_raw',
            lambda msg: self.image_cb('tb3_1', msg),
            10
        )
        self.img_sub_2 = self.create_subscription(
            Image, '/tb3_2/camera/image_raw',
            lambda msg: self.image_cb('tb3_2', msg),
            10
        )

        # QR 포인트 좌표 (예시 값, 실제 환경에 맞게 수정 필요)
        # 1~4번 박스 앞 좌표
        self.qr_points = {
            1: (-1.8, -2.2, 0.0),
            2: ( 1.8, -2.2, math.pi),
            3: ( 1.8,  2.2, math.pi/2),
            4: (-1.8,  2.2, -math.pi/2),
        }

    def send_high_level_cmd(self, robot: str, cmd: str):
        if robot not in self.cmd_pub:
            self.get_logger().error(f'Unknown robot {robot}')
            return
        msg = String()
        msg.data = cmd
        self.cmd_pub[robot].publish(msg)
        self.get_logger().info(f'[{robot}] high_level_cmd: {cmd}')

    def send_goal_to_qr(self, robot: str, qr_index: int):
        """QR 번호에 해당하는 좌표로 goal 송신"""
        if robot not in self.goal_pub:
            self.get_logger().error(f'Unknown robot {robot}')
            return
        if qr_index not in self.qr_points:
            self.get_logger().error(f'Unknown QR index {qr_index}')
            return

        x, y, yaw = self.qr_points[qr_index]
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'map'
        ps.pose.position.x = x
        ps.pose.position.y = y

        # yaw -> quaternion
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
            self.get_logger().error(f'Unknown robot {robot}')
            return
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        self.vel_pub[robot].publish(msg)

    def set_window(self, window):
        """MainWindow 객체를 나중에 주입하기 위한 함수"""
        self.window = window

    def state_cb(self, robot: str, msg: String):
        self.robot_state[robot] = msg.data

    def image_cb(self, robot: str, msg: Image):
        # GUI가 아직 연결 안 되어 있으면 무시
        if self.window is None:
            return

        """
        초기 테스트 때는 항상 카메라 표시,
        BehaviorManager 완성 후 다시 주석 해제

        # 해당 로봇의 상태가 IDLE이면 카메라 갱신 안 함 (대기 중일 때는 부하 줄이기)
        if self.robot_state.get(robot, 'IDLE') == 'IDLE':
            return
        """

        # MainWindow에서 현재 어떤 로봇이 선택되어 있는지 확인
        active_robot = getattr(self.window, 'active_robot', None)
        if active_robot != robot:
            # 선택되지 않은 로봇이면 카메라 갱신 안 함
            return

        # 여기까지 왔으면: '선택된 + 동작중인' 로봇 → 카메라 갱신
        try:
            cv_img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge 변환 실패: {e}')
            return

        # OpenCV BGR → Qt QImage
        h, w, ch = cv_img.shape
        bytes_per_line = ch * w
        qt_img = QImage(
            cv_img.data, w, h, bytes_per_line,
            QImage.Format.Format_BGR888
        )

        # Qt 스레드에서 QLabel 업데이트 (QueuedConnection)
        self.window.update_camera(robot, qt_img)


class MainWindow(QtWidgets.QDialog):
    def __init__(self, bridge: ControlBridge):
        super().__init__()
        # UI 파일 로드
        uic.loadUi(
            '/home/polestar3/ros2_ws/src/tb3_security_system/ui/tb3_control.ui',
            self
        )

        self.bridge = bridge
        self.active_robot = 'tb3_1'  # 기본 선택 로봇

        # ---------------------- ★ 스케일링용 초기값 저장 ★ ----------------------
        # 기준 크기(디자이너에서 만든 원래 폼 크기)
        self._base_size = self.size()
        # 모든 자식 위젯의 원래 geometry 기록
        self._orig_geometries = {}
        for w in self.findChildren(QtWidgets.QWidget):
            if w is self:
                continue
            self._orig_geometries[w] = w.geometry()
        # ----------------------------------------------------------------------

        # 상단에 현재 로봇 표시용 라벨
        if hasattr(self, 'label_active'):
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

        # 정선순찰 (btn_start를 재활용)
        self.btn_start.clicked.connect(self.on_seq_patrol)

        # 난선순찰/복귀 버튼은 UI에 있다고 가정
        if hasattr(self, 'btn_patrol_random'):
            self.btn_patrol_random.clicked.connect(self.on_random_patrol)
        if hasattr(self, 'btn_return'):
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

    # ---------------------- ★ 창 리사이즈 시 전체 스케일링 ★ ----------------------
    def resizeEvent(self, event):
        super().resizeEvent(event)
        if not self._base_size.width() or not self._base_size.height():
            return

        # 가로/세로 비율 계산
        sx = self.width() / self._base_size.width()
        sy = self.height() / self._base_size.height()
        # 한쪽이 너무 찌그러지지 않도록 더 작은 쪽 기준으로 스케일
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
        if hasattr(self, 'label_active'):
            self.label_active.setText(f'현재 로봇: {self.active_robot}')

    def log(self, msg: str):
        """간단 로그 출력 (선택된 로봇에 따라 text_log1/2에 출력)"""
        if self.active_robot == 'tb3_1' and hasattr(self, 'text_log1'):
            self.text_log1.append(msg)
        elif self.active_robot == 'tb3_2' and hasattr(self, 'text_log2'):
            self.text_log2.append(msg)

    # ------------------------
    # 버튼 콜백
    # ------------------------
    def on_switch_robot(self):
        self.active_robot = 'tb3_2' if self.active_robot == 'tb3_1' else 'tb3_1'
        self.update_active_label()
        self.log(f'제어 대상 전환: {self.active_robot}')

    def on_qr_button(self, idx: int):
        # 고수준 명령: "GOTO_QR1" 등
        cmd = f'GOTO_QR{idx}'
        self.bridge.send_high_level_cmd(self.active_robot, cmd)
        # behavior_manager 에서 이 명령을 받아 내부적으로 cmd_goal 전송
        self.log(f'{self.active_robot} → {idx}번 포인트로 이동 명령 ({cmd})')

    def on_qr_scan(self):
        # 현재 위치에서 스캔 후 다음 포인트로 이동
        self.bridge.send_high_level_cmd(self.active_robot, 'QR_SCAN_NEXT')
        self.log(f'{self.active_robot} → QR 스캔 + 다음 포인트 이동 명령')

    def on_buzzer(self):
        # 부저 동작: 현재 goal 취소 + 제자리 회전 + 부저
        self.bridge.send_high_level_cmd(self.active_robot, 'BUZZER')
        self.log(f'{self.active_robot} → 부저 동작 명령')

    def on_seq_patrol(self):
        # 정선순찰 시작
        self.bridge.send_high_level_cmd(self.active_robot, 'SEQ_PATROL_START')
        self.log(f'{self.active_robot} → 정선순찰 시작')

    def on_random_patrol(self):
        # 난선순찰 시작
        self.bridge.send_high_level_cmd(self.active_robot, 'RANDOM_PATROL_START')
        self.log(f'{self.active_robot} → 난선순찰 시작')

    def on_return_home(self):
        # 복귀
        self.bridge.send_high_level_cmd(self.active_robot, 'RETURN_HOME')
        self.log(f'{self.active_robot} → 복귀 명령')

    def on_estop(self):
        # 두 로봇 모두 비상정지
        self.bridge.send_high_level_cmd('tb3_1', 'EMERGENCY_STOP')
        self.bridge.send_high_level_cmd('tb3_2', 'EMERGENCY_STOP')
        self.log('비상정지 명령 전송 (두 로봇 모두)')

    # ------------------------
    # 수동 조종 관련
    # ------------------------
    def set_manual_cmd(self, vx, vy, wz):
        self.manual_cmd = (vx, vy, wz)
        if not self.manual_timer.isActive():
            self.manual_timer.start()
        # 수동 시작 시, nav2 goal은 behavior_manager에서 cancel하도록 설계 가능

    def stop_manual_cmd(self):
        self.manual_cmd = (0.0, 0.0, 0.0)
        self.manual_timer.stop()
        # 마지막으로 0 속도 한 번 보내기
        self.bridge.send_manual_vel(self.active_robot, 0.0, 0.0, 0.0)

    def on_manual_timer(self):
        vx, vy, wz = self.manual_cmd
        self.bridge.send_manual_vel(self.active_robot, vx, vy, wz)

    @QtCore.pyqtSlot(str, QImage)
    def update_camera(self, robot: str, img: QImage):
        """
        ControlBridge.image_cb에서 호출하는 슬롯.
        선택된 + 동작중 로봇의 이미지만 갱신.
        tb3_1은 label_cam1, tb3_2는 label_cam2에 표시하는 예시입니다.
        """
        pix = QPixmap.fromImage(img)

        if robot == 'tb3_1':
            if hasattr(self, 'label_cam1'):
                self.label_cam1.setPixmap(pix)
        elif robot == 'tb3_2':
            if hasattr(self, 'label_cam2'):
                self.label_cam2.setPixmap(pix)


# ------------------------
# rclpy 스레드 + Qt 실행
# ------------------------
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
    bridge.set_window(win)
    win.show()
    ret = app.exec_()

    executor.shutdown()
    bridge.destroy_node()
    rclpy.shutdown()
    sys.exit(ret)


if __name__ == '__main__':
    main()
