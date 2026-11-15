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
    execution_dir = get_package_share_directory('bagger_execution')
    control_dir = get_package_share_directory('bagger_control')
    target_detection_dir = get_package_share_directory('target_detection')
    target_position_estimation_dir = get_package_share_directory('target_position_estimation')

    use_nav2 = LaunchConfiguration('use_nav2')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use the Gazebo simulation clock',
    )

    declare_use_nav2_cmd = DeclareLaunchArgument(
        'use_nav2',
        default_value='true',
        description='Whether to launch Nav2 stack',
    )

    twist_mux_config = os.path.join(bringup_dir, 'config', 'twist_mux.yaml')

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_config, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel_out', '/cmd_vel')],
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_dir, 'launch', 'gazebo.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(navigation_dir, 'launch', 'navigation.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(use_nav2)
    )
    
    execution_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(execution_dir, 'launch', 'execution.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    bucket_wheel_mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(control_dir, 'launch', 'bucket_wheel_mux.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(control_dir, 'launch', 'controllers.launch.py'))
    )

    target_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(target_detection_dir, 'launch', 'target_detection.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    target_position_estimation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(target_position_estimation_dir, 'launch', 'target_position_estimation.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_use_nav2_cmd,
        gazebo_launch,
        navigation_launch,
        twist_mux_node,
        execution_launch,
        bucket_wheel_mux_launch,
        controllers_launch,
        target_detection_launch,
        target_position_estimation_launch,
    ])
