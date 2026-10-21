from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    orchestrator_node = Node(
        package='bagger_autonomy',
        executable='orchestrator',
        name='orchestrator',
        output='screen',
    )

    return LaunchDescription([
        orchestrator_node,
    ])
