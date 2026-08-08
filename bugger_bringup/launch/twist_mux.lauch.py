import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'bugger_bringup'

    pkg_share = get_package_share_directory(package_name)

    twist_mux_config = os.path.join(pkg_share, 'config', 'twist_mux.yaml')

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_config],
        remappings=[('/cmd_vel_out', '/cmd_vel')],
    )

    return LaunchDescription([
        twist_mux_node,
    ])
