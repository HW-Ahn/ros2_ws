# 파일: tb3_security_system/launch/real_two_robots.launch.py

from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():

    # tb3_1용 노드 그룹
    tb3_1_nodes = GroupAction([
        PushRosNamespace('tb3_1'),
        Node(package='tb3_security_system', executable='simple_navigator', name='simple_navigator', output='screen'),
        Node(package='tb3_security_system', executable='behavior_manager', name='behavior_manager', output='screen'),
        Node(package='tb3_security_system', executable='goal_manager', name='goal_manager', output='screen'),
        Node(package='tb3_security_system', executable='obstacle_monitor', name='obstacle_monitor', output='screen'),
    ])

    # tb3_2용 노드 그룹 (2번 로봇이 아직 없다면 나중에만 사용해도 됨)
    tb3_2_nodes = GroupAction([
        PushRosNamespace('tb3_2'),
        Node(package='tb3_security_system', executable='simple_navigator', name='simple_navigator', output='screen'),
        Node(package='tb3_security_system', executable='behavior_manager', name='behavior_manager', output='screen'),
        Node(package='tb3_security_system', executable='goal_manager', name='goal_manager', output='screen'),
        Node(package='tb3_security_system', executable='obstacle_monitor', name='obstacle_monitor', output='screen'),
    ])

    # GUI 노드는 네임스페이스 없이 실행 (내부에서 /tb3_1, /tb3_2로 토픽 분기)
    gui_node = Node(
        package='tb3_security_system',
        executable='gui_main',
        output='screen',
        parameters=[{
            'use_compressed': True,
            'robot1_image_topic': '/tb15/image_raw/compressed',
            'robot2_image_topic': '/tb16/image_raw/compressed'
        }]
    )

    return LaunchDescription([
        tb3_1_nodes,
        tb3_2_nodes,
        gui_node,
    ])
