# 파일: ros2_ws/src/tb3_security_system/launch/spawn_world_and_boxes.launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose',
            '-s', 'libgazebo_ros_factory.so',
            '/usr/share/gazebo-11/worlds/empty.world'
        ],
        output='screen'
    )

    # spawn boxes after small delay
    def spawn_box_node(name, model_path, x, y, z):
        return Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-file', model_path, '-entity', name, '-x', str(x), '-y', str(y), '-z', str(z)],
            output='screen'
        )

    # spawn robots
    def spawn_robot(name, model_path, x, y, z):
        return Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[''
              '-file', model_path,
              '-entity', name,
              '-robot_namespace', f'/{name}',
              '-x', str(x),
              '-y', str(y),
              '-z', str(z)],
            output='screen'
        )

    # LaunchDescription
    return LaunchDescription([
        gazebo,
        TimerAction(period=1.5, actions=[
            spawn_box_node('qr_box', '/home/polestar3/.gazebo/models/qr_box/model.sdf', -1.8, -2.7, 0.125),
            spawn_box_node('qr_box_2', '/home/polestar3/.gazebo/models/qr_box_2/model.sdf',  1.8, -2.7, 0.125),
            spawn_box_node('qr_box_3', '/home/polestar3/.gazebo/models/qr_box_3/model.sdf',  1.8,  2.7, 0.125),
            spawn_box_node('qr_box_4', '/home/polestar3/.gazebo/models/qr_box_4/model.sdf', -1.8,  2.7, 0.125),
        ]),
        # spawn two robots (names tb3_1, tb3_2)
        TimerAction(period=2.5, actions=[
            spawn_robot('tb3_1', '/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger_cam/model.sdf', -2.4, -0.4, 0.0),
            spawn_robot('tb3_2', '/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger_cam/model.sdf', -2.4, 0.4, 0.0),
        ]),
        # after robots spawned, start goal_manager and obstacle_monitor under each namespace
        TimerAction(period=5.0, actions=[
            Node(package='tb3_security_system', executable='goal_manager', namespace='tb3_1', output='screen'),
            Node(package='tb3_security_system', executable='obstacle_monitor', namespace='tb3_1', output='screen'),
            Node(package='tb3_security_system', executable='goal_manager', namespace='tb3_2', output='screen'),
            Node(package='tb3_security_system', executable='obstacle_monitor', namespace='tb3_2', output='screen'),
        ])
    ])
