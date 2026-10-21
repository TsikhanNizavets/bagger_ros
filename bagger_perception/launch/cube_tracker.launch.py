import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use the Gazebo simulation clock',
    )

    cube_boards_path = DeclareLaunchArgument(
        'cube_boards_path',
        default_value=os.path.join('/tmp', 'cube_boards_3d.yaml'),      # if heaps spawning is disabled, yaml is not generated and it gonna crash
        description='Path to cube boards YAML',
    )
    
    declare_camera_debug_view_cmd = DeclareLaunchArgument(
        'camera_debug_view',
        default_value='false',
        description='Whether to start camera debug view',
    )

    node_cube_tracker = Node(
        package='bagger_perception',
        executable='cube_tracker',
        parameters=[{
            'use_sim_time': use_sim_time,
            'cube_boards_path': LaunchConfiguration('cube_boards_path'),
        }],
        output='screen',
    )

    node_camera_debug_view = Node(
            package='bagger_perception',
            executable='camera_debug_view',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('camera_debug_view')),
            output='screen',
        )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        cube_boards_path,
        declare_camera_debug_view_cmd,
        node_cube_tracker,
        node_camera_debug_view,
    ])
