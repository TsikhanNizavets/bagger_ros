import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_share = get_package_share_directory('bagger_gazebo')

    cube_boards_path = LaunchConfiguration('cube_boards_path')

    cube_boards_path = DeclareLaunchArgument(
        'cube_boards_path',
        default_value=os.path.join('/tmp', 'cube_boards_3d.yaml'),      # if heaps spawning is disabled, yaml is not generated and it gonna crash
        description='Path to cube boards YAML',
    )
    
    declare_camera_debug_view_cmd = DeclareLaunchArgument(
        'camera_debug_view',
        default_value='false',
        description='Wheter to start camera debug view',
    )

    node_cube_tracker = Node(
        package='bagger_perception',
        executable='cube_tracker',
        parameters=[{
            'cube_boards_path': LaunchConfiguration('cube_boards_path'),
        }],
        output='screen',
    )

    node_camera_debug_view = Node(
            package='bagger_perception',
            executable='camera_debug_view',
            condition=IfCondition(LaunchConfiguration('camera_debug_view')),
            output='screen',
        )

    return LaunchDescription([
        cube_boards_path,
        declare_camera_debug_view_cmd,
        node_cube_tracker,
        node_camera_debug_view,
    ])