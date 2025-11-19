import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_tb3 = FindPackageShare('tb3_security_system')

    # -------------------------------
    # 1. Gazebo 월드 + TurtleBot3 2대 스폰
    # -------------------------------
    # spawn_world_and_boxes.launch.py가 /tb3_1, /tb3_2를 스폰한다고 가정
    world_launch = PathJoinSubstitution([
        pkg_tb3,
        'launch',
        'spawn_world_and_boxes.launch.py'
    ])

    gazebo_and_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(world_launch)
    )

    nodes = [gazebo_and_spawn]

    # -------------------------------
    # 2. 각 로봇 별 노드 실행 (Nav2 제거 -> simple_navigator 추가)
    # -------------------------------
    # tb3_1, tb3_2 각각에 대해 behavior_manager와 simple_navigator를 띄웁니다.

    for ns in ['tb3_1', 'tb3_2']:
        # (1) Behavior Manager: GUI 명령 수신 및 상태 관리
        nodes.append(Node(
            package='tb3_security_system',
            executable='behavior_manager',
            namespace=ns,
            output='screen',
            parameters=[{'use_sim_time': True}]
        ))

        # (2) Simple Navigator: cmd_goal을 받아 실제 주행 담당 (Nav2 대체)
        nodes.append(Node(
            package='tb3_security_system',
            executable='simple_navigator',
            namespace=ns,
            output='screen',
            parameters=[{'use_sim_time': True}]
        ))

    # -------------------------------
    # 3. GUI 실행
    # -------------------------------
    # GUI는 로봇 네임스페이스 밖에서 실행되지만,
    # 내부적으로 /tb3_1/high_level_cmd 등을 발행해야 합니다.
    gui_node = Node(
        package='tb3_security_system',
        executable='gui_main', # setup.py에 등록된 이름 확인 필요 (tb3_gui.py)
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    nodes.append(gui_node)

    return LaunchDescription(nodes)
