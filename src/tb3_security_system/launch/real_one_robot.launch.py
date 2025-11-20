from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():

    # --- 로봇 네임스페이스 tb3_1 ---
    tb3_1_nodes = GroupAction([
        PushRosNamespace('tb3_1'),

        Node(package='tb3_security_system',
             executable='simple_navigator',
             name='simple_navigator',
             output='screen'),

        Node(package='tb3_security_system',
             executable='behavior_manager',
             name='behavior_manager',
             output='screen'),

        Node(package='tb3_security_system',
             executable='goal_manager',
             name='goal_manager',
             output='screen'),

        Node(package='tb3_security_system',
             executable='obstacle_monitor',
             name='obstacle_monitor',
             output='screen'),
    ])

    # --- GUI 노드 ---
    gui_node = Node(
        package='tb3_security_system',
        executable='gui_main',
        output='screen',
        parameters=[{
            'use_compressed': True,
            'robot1_image_topic': '/tb3_1/image_raw/compressed'
        }]
    )

    return LaunchDescription([
        tb3_1_nodes,
        gui_node
    ])
