# 파일: tb3_security_system/launch/sim_two_robots.launch.py

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, GroupAction
from launch_ros.actions import Node, PushRosNamespace
import os

def generate_launch_description():

    # 1. Gazebo GUI 실행 (empty.world + factory 플러그인)
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose',
            '-s', 'libgazebo_ros_factory.so',
            '/usr/share/gazebo-11/worlds/empty.world'
        ],
        output='screen'
    )

    # 2. TurtleBot3 Gazebo용 SDF 모델 경로 (burger_cam 사용)
    tb3_model_sdf = '/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger_cam/model.sdf'

    # 3. 로봇 스폰 노드 (tb3_1, tb3_2)
    def spawn_robot(name, x, y, z):
        return Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-file', tb3_model_sdf,
                '-entity', name,
                '-robot_namespace', f'/{name}',
                '-x', str(x),
                '-y', str(y),
                '-z', str(z)
            ],
            output='screen'
        )

    spawn_tb3_1 = spawn_robot('tb3_1', -2.4, -0.4, 0.0)
    spawn_tb3_2 = spawn_robot('tb3_2', -2.4,  0.4, 0.0)

    # 4. QR 박스 스폰 ( ~/.gazebo/models 아래 모델 사용 )
    def spawn_box(name, model_rel_path, x, y, z):
        model_path = os.path.expanduser(model_rel_path)
        return Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-file', model_path,
                '-entity', name,
                '-x', str(x),
                '-y', str(y),
                '-z', str(z)
            ],
            output='screen'
        )

    qr_boxes = [
        ('qr_box', '~/.gazebo/models/qr_box/model.sdf',   -1.8, -2.7, 0.125),  # QR1
        ('qr_box_2', '~/.gazebo/models/qr_box_2/model.sdf',  1.8, -2.7, 0.125),  # QR2
        ('qr_box_3', '~/.gazebo/models/qr_box_3/model.sdf',  1.8,  2.7, 0.125),  # QR3
        ('qr_box_4', '~/.gazebo/models/qr_box_4/model.sdf', -1.8,  2.7, 0.125),  # QR4
    ]

    spawn_boxes = [
        spawn_box(name, model_path, x, y, z)
        for (name, model_path, x, y, z) in qr_boxes
    ]

    # 5. 각 로봇 네임스페이스에 우리 노드들 띄우기
    tb3_1_nodes = GroupAction([
        PushRosNamespace('tb3_1'),
        Node(package='tb3_security_system', executable='simple_navigator', name='simple_navigator', output='screen'),
        Node(package='tb3_security_system', executable='behavior_manager', name='behavior_manager', output='screen'),
        Node(package='tb3_security_system', executable='goal_manager', name='goal_manager', output='screen'),
        Node(package='tb3_security_system', executable='obstacle_monitor', name='obstacle_monitor', output='screen'),
    ])

    tb3_2_nodes = GroupAction([
        PushRosNamespace('tb3_2'),
        Node(package='tb3_security_system', executable='simple_navigator', name='simple_navigator', output='screen'),
        Node(package='tb3_security_system', executable='behavior_manager', name='behavior_manager', output='screen'),
        Node(package='tb3_security_system', executable='goal_manager', name='goal_manager', output='screen'),
        Node(package='tb3_security_system', executable='obstacle_monitor', name='obstacle_monitor', output='screen'),
    ])

    # 6. GUI 실행
    gui_node = Node(
        package='tb3_security_system',
        executable='gui_main',
        output='screen'
    )

    # 7. LaunchDescription (순서/딜레이 조절)
    return LaunchDescription([
        gazebo,

        # 박스 먼저 스폰
        TimerAction(period=2.0, actions=spawn_boxes),

        # 로봇 스폰
        TimerAction(period=4.0, actions=[
            spawn_tb3_1,
            spawn_tb3_2,
        ]),

        # 우리 노드 + GUI 실행
        TimerAction(period=6.0, actions=[
            tb3_1_nodes,
            tb3_2_nodes,
            gui_node,
        ]),
    ])
