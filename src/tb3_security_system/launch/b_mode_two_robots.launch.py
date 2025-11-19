# 파일: tb3_security_system/launch/b_mode_two_robots.launch.py

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    pkg_tb3 = FindPackageShare('tb3_security_system')

    # 기존 월드 + 박스 + 로봇 스폰
    world_launch = PathJoinSubstitution([
        pkg_tb3,
        'launch',
        'spawn_world_and_boxes.launch.py'
    ])
    gazebo_and_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(world_launch)
    )

    nodes = [gazebo_and_spawn]

    for ns in ['tb3_1', 'tb3_2']:
        nodes.append(
            Node(
                package='tb3_security_system',
                executable='behavior_manager',
                namespace=ns,
                output='screen'
            )
        )
        nodes.append(
            Node(
                package='tb3_security_system',
                executable='simple_navigator',
                namespace=ns,
                output='screen'
            )
        )

    gui_node = Node(
        package='tb3_security_system',
        executable='gui_main',
        output='screen'
    )

    nodes.append(gui_node)

    return LaunchDescription(nodes)
