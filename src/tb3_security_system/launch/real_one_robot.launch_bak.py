from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():

    # ------------ tb3_1 전용 노드 그룹 ------------
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

    # ------------ 카메라 republish (raw → compressed) ------------
    # v4l2_camera는 raw를 내보내므로 compressed로 다시 전송
    camera_relay = Node(
        package='image_transport',
        executable='republish',
        name='camera_relay_tb3_1',
        output='screen',
        arguments=['raw', 'compressed'],
        remappings=[
            ('in', '/tb3_1/image_raw'),
            ('out', '/tb3_1/camera/image_raw/compressed'),
        ]
    )

    # ------------ GUI 노드 ------------
    gui_node = Node(
        package='tb3_security_system',
        executable='gui_main',
        name='gui_main',
        output='screen',
        parameters=[{
            'use_compressed': True,
            # GUI는 tb3_1 카메라 사용
            'robot1_image_topic': '/tb3_1/image_raw/compressed'
        }]
    )

    return LaunchDescription([
        tb3_1_nodes,
        camera_relay,
        gui_node
    ])
