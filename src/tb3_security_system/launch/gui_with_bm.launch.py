from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # GUI 실행
        Node(
            package='tb3_security_system',
            executable='gui_main',
            output='screen'
        ),

        # tb3_1 behavior manager
        Node(
            package='tb3_security_system',
            executable='behavior_manager',
            namespace='tb3_1',
            output='screen'
        ),

        # tb3_2 behavior manager
        Node(
            package='tb3_security_system',
            executable='behavior_manager',
            namespace='tb3_2',
            output='screen'
        ),
    ])
