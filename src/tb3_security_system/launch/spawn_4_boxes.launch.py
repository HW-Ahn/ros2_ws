from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    boxes = [
        ("qr_box", "/home/polestar3/.gazebo/models/qr_box/model.sdf", -1.8, -2.7, 0.125),
        ("qr_box_2", "/home/polestar3/.gazebo/models/qr_box_2/model.sdf", 1.8, -2.7, 0.125),
        ("qr_box_3", "/home/polestar3/.gazebo/models/qr_box_3/model.sdf", 1.8, 2.7, 0.125),
        ("qr_box_4", "/home/polestar3/.gazebo/models/qr_box_4/model.sdf", -1.8, 2.7, 0.125),
    ]

    nodes = []

    for name, path, x, y, z in boxes:
        nodes.append(
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', name,
                    '-file', path,
                    '-x', str(x),
                    '-y', str(y),
                    '-z', str(z)
                ],
                output='screen'
            )
        )

    return LaunchDescription(nodes)
