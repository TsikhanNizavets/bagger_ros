from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'bugger_manipulation'

    container_unloader_node = Node(             
        package=package_name,
        executable='container_unloader',
        name='container_unloader',
        output='screen',
    )

    return LaunchDescription([
        container_unloader_node,
    ])
