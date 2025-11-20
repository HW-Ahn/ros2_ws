from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace='tb3_1',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'slam_params_file': '/opt/ros/humble/share/slam_toolbox/config/mapper_params_online_async.yaml'
            }],
            remappings=[
                ('/scan', '/tb3_1/scan'),
                ('/tf', '/tf'),
                ('/tf_static', '/tf_static')
            ]
        ),
    ])
