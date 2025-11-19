# 파일: tb3_security_system/scripts/tb3_gui.py

import sys
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from PyQt5 import QtWidgets, uic

class GuiRosNode(Node):
    def __init__(self):
        super().__init__('tb3_gui_node')
        # 토픽 이름이 behavior_manager와 일치해야 합니다.
        self.publishers = {
            'tb3_1': self.create_publisher(String, '/tb3_1/high_level_cmd', 10),
            'tb3_2': self.create_publisher(String, '/tb3_2/high_level_cmd', 10),
        }

    def send_command(self, robot_name: str, cmd: str):
        if robot_name not in self.publishers:
            return
        msg = String()
        msg.data = cmd
        self.publishers[robot_name].publish(msg)
        self.get_logger().info(f'[{robot_name}] Send: {cmd}')

class MainDialog(QtWidgets.QDialog):
    def __init__(self, ros_node: GuiRosNode, ui_path: str, parent=None):
        super().__init__(parent)
        uic.loadUi(ui_path, self)
        self.ros_node = ros_node
        self.current_robot = 'tb3_1'
        self.update_title()

        # --- 1. 방향키 (수동 조작) 매핑 ---
        # 버튼을 누르면 해당 방향으로 속도 명령을 보냄
        self.btn_up.clicked.connect(lambda: self.send_behavior('MANUAL_FWD'))
        self.btn_down.clicked.connect(lambda: self.send_behavior('MANUAL_BWD'))
        self.btn_left.clicked.connect(lambda: self.send_behavior('MANUAL_LEFT'))
        self.btn_right.clicked.connect(lambda: self.send_behavior('MANUAL_RIGHT'))

        # --- 2. 정지 버튼 (모든 동작 멈춤) ---
        # btn_stop 혹은 가운데 버튼 등을 정지로 사용
        self.btn_stop.clicked.connect(lambda: self.send_behavior('STOP'))

        # --- 3. QR 및 순찰 기능 ---
        self.btn_qr1.clicked.connect(lambda: self.send_behavior('GOTO_QR1'))
        self.btn_qr2.clicked.connect(lambda: self.send_behavior('GOTO_QR2'))
        self.btn_qr3.clicked.connect(lambda: self.send_behavior('GOTO_QR3'))
        self.btn_qr4.clicked.connect(lambda: self.send_behavior('GOTO_QR4'))

        self.btn_qrscan.clicked.connect(lambda: self.send_behavior('QR_SCAN_NEXT'))
        self.btn_start.clicked.connect(lambda: self.send_behavior('SEQ_PATROL_START'))
        self.btn_patrol_random.clicked.connect(lambda: self.send_behavior('RANDOM_PATROL_START'))
        self.btn_return.clicked.connect(lambda: self.send_behavior('RETURN_HOME'))

        # --- 4. 유틸리티 ---
        self.btn_buzzer.clicked.connect(lambda: self.send_behavior('BUZZER'))
        self.btn_estop.clicked.connect(lambda: self.send_behavior('EMERGENCY_STOP'))
        self.btn_switch.clicked.connect(self.on_switch_robot)

        self.text_log1.append('GUI Started.')

    def update_title(self):
        self.setWindowTitle(f'Control: {self.current_robot}')

    def send_behavior(self, cmd: str):
        self.text_log1.append(f'[{self.current_robot}] {cmd}')
        # 자동 스크롤
        sb = self.text_log1.verticalScrollBar()
        sb.setValue(sb.maximum())
        self.ros_node.send_command(self.current_robot, cmd)

    def on_switch_robot(self):
        # 단순히 제어 대상만 변경 (HANDOVER 명령은 별도 로직 필요 시 추가)
        self.current_robot = 'tb3_2' if self.current_robot == 'tb3_1' else 'tb3_1'
        self.update_title()
        self.text_log1.append(f'Switched to {self.current_robot}')

def main():
    rclpy.init()
    ros_node = GuiRosNode()
    app = QtWidgets.QApplication(sys.argv)
    # 경로 주의
    ui_path = '/home/polestar3/ros2_ws/src/tb3_security_system/resource/tb3_gui.ui'
    dlg = MainDialog(ros_node, ui_path)
    dlg.show()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(ros_node)
    th = threading.Thread(target=executor.spin, daemon=True)
    th.start()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
