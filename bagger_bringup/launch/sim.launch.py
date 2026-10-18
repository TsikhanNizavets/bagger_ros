import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    gazebo_dir = get_package_share_directory('bagger_gazebo')
    navigation_dir = get_package_share_directory('bagger_navigation')
    bringup_dir = get_package_share_directory('bagger_bringup')
    manipulation_dir = get_package_share_directory('bagger_manipulation')
    control_dir = get_package_share_directory('bagger_control')
    perception_dir = get_package_share_directory('bagger_perception')

    use_nav2 = LaunchConfiguration('use_nav2')

    declare_use_nav2_cmd = DeclareLaunchArgument(
        'use_nav2',
        default_value='true',
        description='Whether to launch Nav2 stack',
    )

    declare_cube_boards_path = DeclareLaunchArgument(
        'cube_boards_path',
        default_value='/tmp/cube_boards_3d.yaml',
    )

    twist_mux_config = os.path.join(bringup_dir, 'config', 'twist_mux.yaml')

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_config],
        remappings=[('/cmd_vel_out', '/cmd_vel')],
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_dir, 'launch', 'gazebo.launch.py')),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(navigation_dir, 'launch', 'navigation.launch.py')),
        condition=IfCondition(use_nav2)
    )
    
    container_unloader_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(manipulation_dir, 'launch', 'container_unloader.launch.py')),
    )

    bucket_wheel_mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(control_dir, 'launch', 'bucket_wheel_mux.launch.py'))
    )

    controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(control_dir, 'launch', 'controllers.launch.py'))
    )

    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(perception_dir, 'launch', 'cube_tracking.launch.py'))
    )

    return LaunchDescription([
        declare_use_nav2_cmd,
        declare_cube_boards_path,
        gazebo_launch,
        navigation_launch,
        twist_mux_node,
        container_unloader_launch,
        bucket_wheel_mux_launch,
        controllers_launch,
        perception_launch,
    ])
