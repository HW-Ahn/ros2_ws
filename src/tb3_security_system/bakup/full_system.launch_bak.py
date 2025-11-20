from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # -------------------------------
    # 경로 설정
    # -------------------------------
    pkg_tb3 = FindPackageShare('tb3_security_system')
    pkg_nav2 = FindPackageShare('nav2_bringup')

    world_launch = PathJoinSubstitution([
        pkg_tb3,
        'launch',
        'spawn_world_and_boxes.launch.py'
    ])

    nav2_params = PathJoinSubstitution([
        pkg_nav2,
        'params',
        'nav2_params.yaml'
    ])

    # -------------------------------
    # Gazebo + TurtleBot3 2대 스폰
    # -------------------------------
    gazebo_and_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(world_launch)
    )

    # -------------------------------
    # Nav2 bringup for tb3_1
    # -------------------------------
    nav2_tb3_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            pkg_nav2,
            'launch',
            'bringup_launch.py'
        ])),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
            'namespace': 'tb3_1',
            'params_file': nav2_params,
            'map': PathJoinSubstitution([pkg_tb3, 'maps', 'my_map.yaml'])
        }.items()
    )

    # -------------------------------
    # Nav2 bringup for tb3_2
    # -------------------------------
    nav2_tb3_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            pkg_nav2,
            'launch',
            'bringup_launch.py'
        ])),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
            'namespace': 'tb3_2',
            'params_file': nav2_params,
            'map': PathJoinSubstitution([pkg_tb3, 'maps', 'my_map.yaml'])
        }.items()
    )

    # -------------------------------
    # BehaviorManager for tb3_1
    # -------------------------------
    behavior_tb3_1 = Node(
        package='tb3_security_system',
        executable='behavior_manager',
        namespace='tb3_1',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # -------------------------------
    # BehaviorManager for tb3_2
    # -------------------------------
    behavior_tb3_2 = Node(
        package='tb3_security_system',
        executable='behavior_manager',
        namespace='tb3_2',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # -------------------------------
    # GUI 실행
    # -------------------------------
    gui_node = Node(
        package='tb3_security_system',
        executable='gui_main',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # -------------------------------
    # Launch Description 반환
    # -------------------------------
    return LaunchDescription([
        gazebo_and_spawn,
        nav2_tb3_1,
        nav2_tb3_2,
        behavior_tb3_1,
        behavior_tb3_2,
        gui_node
    ])
