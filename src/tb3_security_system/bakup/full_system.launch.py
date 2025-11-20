from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, GroupAction
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # ---------------------------------------
    # 1) Gazebo 실행
    # ---------------------------------------
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose',
            '-s', 'libgazebo_ros_factory.so',
            '/usr/share/gazebo-11/worlds/empty.world'
        ],
        output='screen'
    )

    # ---------------------------------------
    # 2) TurtleBot3 모델 파일 경로
    # ---------------------------------------
    tb3_model = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf', 'turtlebot3_waffle.urdf'
    )

    # ---------------------------------------
    # 3) 로봇 스폰
    # ---------------------------------------
    spawn_tb3_1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'tb3_1',
            '-file', tb3_model,
            '-x', '-2.4',
            '-y', '-0.4',
            '-z', '0.01'
        ],
        output='screen'
    )

    spawn_tb3_2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'tb3_2',
            '-file', tb3_model,
            '-x', '-2.4',
            '-y', '0.4',
            '-z', '0.01'
        ],
        output='screen'
    )

    # ---------------------------------------
    # 4) QR 박스 스폰
    # ---------------------------------------
    qr_spawn_nodes = []
    qr_boxes = [
        ('qr1', '~/.gazebo/models/qr_box/model.sdf', -1.8, -2.7),
        ('qr2', '~/.gazebo/models/qr_box_2/model.sdf', 1.8, -2.7),
        ('qr3', '~/.gazebo/models/qr_box_3/model.sdf', 1.8, 2.7),
        ('qr4', '~/.gazebo/models/qr_box_4/model.sdf', -1.8, 2.7),
    ]

    for name, model, x, y in qr_boxes:
        qr_spawn_nodes.append(
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-file', os.path.expanduser(model),
                    '-entity', name,
                    '-x', str(x),
                    '-y', str(y),
                    '-z', '0.125'
                ],
                output='screen'
            )
        )

    # ---------------------------------------
    # 5) 두 로봇의 ROS2 노드 그룹
    # ---------------------------------------
    tb3_1_nodes = GroupAction([
        PushRosNamespace('tb3_1'),
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             parameters=[{'use_sim_time': True},
                         {'robot_description': open(tb3_model).read()}]),
        Node(package='tb3_security_system', executable='simple_navigator'),
        Node(package='tb3_security_system', executable='behavior_manager'),
        Node(package='tb3_security_system', executable='goal_manager'),
    ])

    tb3_2_nodes = GroupAction([
        PushRosNamespace('tb3_2'),
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             parameters=[{'use_sim_time': True},
                         {'robot_description': open(tb3_model).read()}]),
        Node(package='tb3_security_system', executable='simple_navigator'),
        Node(package='tb3_security_system', executable='behavior_manager'),
        Node(package='tb3_security_system', executable='goal_manager'),
    ])

    # ---------------------------------------
    # 6) GUI 실행
    # ---------------------------------------
    gui_node = Node(
        package='tb3_security_system',
        executable='gui_main',
        output='screen'
    )

    # ---------------------------------------
    # LaunchDescription (타이머 포함)
    # ---------------------------------------
    return LaunchDescription([
        gazebo,

        TimerAction(period=2.0, actions=[
            *qr_spawn_nodes
        ]),

        TimerAction(period=4.0, actions=[
            spawn_tb3_1,
            spawn_tb3_2,
        ]),

        TimerAction(period=6.0, actions=[
            tb3_1_nodes,
            tb3_2_nodes,
            gui_node,
        ]),
    ])
