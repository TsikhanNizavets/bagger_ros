import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'bagger_control'

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use the Gazebo simulation clock',
    )

    pkg_share = get_package_share_directory(package_name)

    bucket_wheel_mux_config = os.path.join(pkg_share, 'config', 'bucket_wheel_mux.yaml')

    bucket_wheel_mux_node = Node(
        package='bagger_control',
        executable='bucket_wheel_mux',
        name='bucket_wheel_mux',
        output='screen',
        parameters=[bucket_wheel_mux_config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        bucket_wheel_mux_node,
    ])
