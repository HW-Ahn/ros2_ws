from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true'
    )

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='tb3_1'
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value='/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml'
    )

    # ------------------------------------------------------------
    #  SLAM Toolbox (Lifecycle Node with namespace)
    # ------------------------------------------------------------
    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        namespace=namespace,
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # ------------------------------------------------------------
    #  Nav2 Controller Server
    # ------------------------------------------------------------
    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        namespace=namespace,
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    # ------------------------------------------------------------
    #  Nav2 Planner Server
    # ------------------------------------------------------------
    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        namespace=namespace,
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    # ------------------------------------------------------------
    #  Behavior Tree Navigator
    # ------------------------------------------------------------
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        namespace=namespace,
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    # ------------------------------------------------------------
    #  Nav2 Lifecycle Manager
    # ------------------------------------------------------------
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'slam_toolbox',
                'controller_server',
                'planner_server',
                'bt_navigator'
            ]
        }]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_namespace,
        declare_params_file,

        slam_node,
        controller_node,
        planner_node,
        bt_navigator_node,
        lifecycle_manager
    ])
