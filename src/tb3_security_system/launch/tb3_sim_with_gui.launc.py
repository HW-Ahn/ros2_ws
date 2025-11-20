from launch import LaunchDescription
from launch.actions import GroupAction, PushRosNamespace
from launch_ros.actions import Node


def generate_launch_description():

    # --- tb3_1 네임스페이스 로봇 노드들 (시뮬용 리매핑 포함) ---
    tb3_1_nodes = GroupAction([
        PushRosNamespace('tb3_1'),

        # 단순 내비게이터
        Node(
            package='tb3_security_system',
            executable='simple_navigator',
            name='simple_navigator',
            output='screen',
            remappings=[
                # 가제보의 /odom, /scan, /cmd_vel 로 직접 연결
                ('odom', '/odom'),
                ('scan', '/scan'),
                ('cmd_vel', '/cmd_vel'),
            ],
        ),

        # 상위 행동 관리자 (움직이는 장애물 감지 포함)
        Node(
            package='tb3_security_system',
            executable='behavior_manager',
            name='behavior_manager',
            output='screen',
            remappings=[
                ('odom', '/odom'),
                ('scan', '/scan'),
                # sound 서비스는 시뮬에서는 없어도 무방
            ],
        ),

        # 필요하다면 goal_manager 도 시뮬에서 같이 사용
        Node(
            package='tb3_security_system',
            executable='goal_manager',
            name='goal_manager',
            output='screen',
            remappings=[
                ('odom', '/odom'),
            ],
        ),

        # obstacle_monitor 를 계속 쓰고 싶다면 /scan 리매핑만 추가
        """Node(
            package='tb3_security_system',
            executable='obstacle_monitor',
            name='obstacle_monitor',
            output='screen',
            remappings=[
                ('scan', '/scan'),
            ],
        ),"""
    ])

    # --- GUI 노드 ---
    # 현재 토픽 리스트에 /tb3_1/image_raw 가 있으므로,
    # 압축 이미지 대신 RAW 이미지를 그대로 사용
    gui_node = Node(
        package='tb3_security_system',
        executable='gui_main',
        output='screen',
        parameters=[{
            'use_compressed': False,              # 압축 사용 안 함
            'robot1_image_topic': '/tb3_1/image_raw',
            'robot2_image_topic': '/tb3_2/image_raw',
        }]
    )

    return LaunchDescription([
        tb3_1_nodes,
        gui_node,
    ])
