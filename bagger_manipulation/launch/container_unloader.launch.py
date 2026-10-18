from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'bagger_manipulation'

    container_unloader_node = Node(             
        package=package_name,
        executable='container_unloader',
        name='container_unloader',
        output='screen',
    )

    dumping_executor_node = Node(
        package=package_name,
        executable='dumping_executor',
        name='dumping_executor',
        output='screen',
    )

    return LaunchDescription([
        container_unloader_node,
        dumping_executor_node,
    ])
