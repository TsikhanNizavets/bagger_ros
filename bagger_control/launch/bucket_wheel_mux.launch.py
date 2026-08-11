import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'bagger_control'

    pkg_share = get_package_share_directory(package_name)

    bucket_wheel_mux_config = os.path.join(pkg_share, 'config', 'bucket_wheel_mux.yaml')

    bucket_wheel_mux_node = Node(
        package='bagger_control',
        executable='bucket_wheel_mux',
        name='bucket_wheel_mux',
        output='screen',
        parameters=[bucket_wheel_mux_config],
    )

    return LaunchDescription([
        bucket_wheel_mux_node,
    ])
