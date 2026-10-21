from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'bagger_manipulation'

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use the Gazebo simulation clock',
    )

    container_unloader_node = Node(             
        package=package_name,
        executable='container_unloader',
        name='container_unloader',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    dumping_executor_node = Node(
        package=package_name,
        executable='dumping_executor',
        name='dumping_executor',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        container_unloader_node,
        dumping_executor_node,
    ])
