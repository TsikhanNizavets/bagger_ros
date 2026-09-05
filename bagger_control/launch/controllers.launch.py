from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster'],
    )

    bucket_wheel_pid_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['bucket_wheel_pid_controller'],
    )

    return LaunchDescription([
        joint_state_broadcaster_spawner,
        bucket_wheel_pid_controller_spawner,
    ])