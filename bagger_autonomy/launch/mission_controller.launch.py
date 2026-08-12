import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'bagger_autonomy'

    pkg_share = get_package_share_directory(package_name)

    mission_controller_node = Node(
        package='bagger_autonomy',
        executable='mission_controller',
        name='mission_controller',
        output='screen',
    )

    return LaunchDescription([
        mission_controller_node,
    ])
