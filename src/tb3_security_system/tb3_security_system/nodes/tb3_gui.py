# 파일: ros2_ws/src/tb3_security_system/tb3_security_system/scripts/tb3_gui.py

import sys
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from PyQt5 import QtWidgets, QtCore, uic


class GuiRosNode(Node):
    """GUI와 연동되는 ROS 노드: 각 로봇의 behavior_cmd 퍼블리셔 관리"""

    def __init__(self):
        super().__init__('tb3_gui_node')
        # 로봇별 behavior_cmd 퍼블리셔
        self.publishers = {
            'tb3_1': self.create_publisher(String, '/tb3_1/behavior_cmd', 10),
            'tb3_2': self.create_publisher(String, '/tb3_2/behavior_cmd', 10),
        }

    def send_command(self, robot_name: str, cmd: str):
        if robot_name not in self.publishers:
            self.get_logger().warn(f'Unknown robot name: {robot_name}')
            return
        msg = String()
        msg.data = cmd
        self.publishers[robot_name].publish(msg)
        self.get_logger().info(f'[GUI] Send command to {robot_name}: {cmd}')


class MainDialog(QtWidgets.QDialog):
    def __init__(self, ros_node: GuiRosNode, ui_path: str, parent=None):
        super().__init__(parent)
        uic.loadUi(ui_path, self)

        self.ros_node = ros_node

        # 현재 선택된 로봇 (초기값 tb3_1)
        self.current_robot = 'tb3_1'
        self.update_title()

        # 버튼 시그널 연결
        self.btn_switch.clicked.connect(self.on_switch_robot)

        self.btn_qr1.clicked.connect(lambda: self.send_behavior('MOVE_QR1'))
        self.btn_qr2.clicked.connect(lambda: self.send_behavior('MOVE_QR2'))
        self.btn_qr3.clicked.connect(lambda: self.send_behavior('MOVE_QR3'))
        self.btn_qr4.clicked.connect(lambda: self.send_behavior('MOVE_QR4'))

        self.btn_qrscan.clicked.connect(lambda: self.send_behavior('QR_SCAN'))

        # 정선순찰 시작/정지 토글 (여기서는 단순히 시작 명령만 보냄, 정지는 ESTOP 또는 STOP_ALL로 처리)
        self.btn_start.clicked.connect(lambda: self.send_behavior('PATROL_FIXED'))

        # 난선순찰 – 예시로 btn_down에 매핑 (원하시면 다른 버튼으로 변경 가능)
        self.btn_down.clicked.connect(lambda: self.send_behavior('PATROL_RANDOM'))

        # 복귀 – 예시로 btn_up에 매핑
        self.btn_up.clicked.connect(lambda: self.send_behavior('RETURN_HOME'))

        # 부저
        self.btn_buzzer.clicked.connect(lambda: self.send_behavior('BUZZER'))

        # 비상정지
        self.btn_estop.clicked.connect(lambda: self.send_behavior('ESTOP'))

        # 종료 버튼 – GUI 종료 + 노드 종료
        self.btn_stop.clicked.connect(self.on_close_clicked)

        # 로그 텍스트 박스
        self.text_log1.append('tb3_security_system GUI 시작')
        self.text_log2.append('초기 선택 로봇: tb3_1')

    # ----------------- 유틸 및 콜백 ----------------- #
    def update_title(self):
        self.setWindowTitle(f'TB3 Security GUI - 현재 선택 로봇: {self.current_robot}')

    def log(self, text: str):
        self.text_log1.append(text)

    def send_behavior(self, cmd: str):
        self.log(f'{self.current_robot} 에 명령: {cmd}')
        self.ros_node.send_command(self.current_robot, cmd)

    def on_switch_robot(self):
        # tb3_1 <-> tb3_2 토글
        self.current_robot = 'tb3_2' if self.current_robot == 'tb3_1' else 'tb3_1'
        self.update_title()
        self.text_log2.append(f'선택 로봇 변경: {self.current_robot}')

    def on_close_clicked(self):
        self.close()

    def closeEvent(self, event):
        # GUI 종료 시 ROS도 함께 종료
        rclpy.shutdown()
        event.accept()


def ros_spin_thread():
    rclpy.spin(rclpy.get_global_executor())


def main():
    rclpy.init()
    ros_node = GuiRosNode()

    app = QtWidgets.QApplication(sys.argv)
    # ui 파일 경로는 실제 위치로 변경해 주세요
    ui_path = '/home/polestar3/ros2_ws/src/tb3_security_system/resource/tb3_gui.ui'
    dlg = MainDialog(ros_node, ui_path)
    dlg.show()

    # rclpy 실행용 스레드 (MultiThreadedExecutor를 쓰고 있다면 그에 맞게 구성)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(ros_node)

    th = threading.Thread(target=executor.spin, daemon=True)
    th.start()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
