from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    bagger_teleop_node = Node(
        package='bagger_teleop',
        executable='teleop_bagger',
        name='teleop_bagger',
        output='screen',
    )

    return LaunchDescription([
        bagger_teleop_node,
    ])
