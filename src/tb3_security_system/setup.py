from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'tb3_security_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # launch 파일
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),

        # 지도 파일
        ('share/' + package_name + '/maps', [
            'tb3_security_system/maps/my_map.yaml',
            'tb3_security_system/maps/my_map.pgm'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='polestar3',
    maintainer_email='polestar3@todo.todo',
    description='TB3 Security System',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'gui_main = tb3_security_system.gui.gui_main:main',
            'simple_navigator = tb3_security_system.nodes.simple_navigator:main',
            'behavior_manager = tb3_security_system.nodes.behavior_manager:main',
            'goal_manager = tb3_security_system.nodes.goal_manager:main',
            'obstacle_monitor = tb3_security_system.nodes.obstacle_monitor:main',

            # 새 노드
            'qr_scanner = tb3_security_system.nodes.qr_scanner:main',
            'return_home = tb3_security_system.nodes.return_home:main',
            'random_patrol = tb3_security_system.nodes.random_patrol:main',
            'seq_patrol = tb3_security_system.nodes.seq_patrol:main',
        ],
    },
)
