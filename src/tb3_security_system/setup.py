from setuptools import find_packages, setup
from glob import glob

package_name = 'tb3_security_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/tb3_security_system']),
        ('share/tb3_security_system', ['package.xml']),
        ('share/tb3_security_system/launch', glob('launch/*.launch.py')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='polestar3',
    maintainer_email='polestar3@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
          'gui_main = tb3_security_system.gui.gui_main:main',
          'goal_manager = tb3_security_system.nodes.goal_manager:main',
          'obstacle_monitor = tb3_security_system.nodes.obstacle_monitor:main',
          'behavior_manager = tb3_security_system.nodes.behavior_manager:main',
          #'patrol_node = tb3_security_system.nodes.patrol_node:main'



        ],
    },
)
