import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/polestar3/ros2_ws/src/install/tb3_security_system'
