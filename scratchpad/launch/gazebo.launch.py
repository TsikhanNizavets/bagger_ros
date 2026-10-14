"""Launch the tagged cube, camera, and ROS camera bridge."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """Load the world and publish its camera images in ROS."""
    package_share = get_package_share_directory('scratchpad')
    world_path = os.path.join(
        package_share, 'worlds', 'plane.sdf'
    )
    gazebo_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
    )

    declare_spawn_heaps_cmd = DeclareLaunchArgument(
        'spawn_heaps',
        default_value='true',
        description='Wheter to spawn heaps',
    )

    node_heaps_spawner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_share, 'launch', 'spawn_heaps.launch.py')),
        condition=IfCondition(LaunchConfiguration('spawn_heaps')),
    )

    node_cube_tracking = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(package_share, 'launch', 'cube_tracking.launch.py')),
        )

    node_inspect_tag_corners = Node(
        package='scratchpad',
        executable='inspect_tag_corners',
        output='screen',
    )

    return LaunchDescription([
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH', os.path.dirname(package_share)
        ),
        declare_spawn_heaps_cmd,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                'gz_args': f'-r "{world_path}"',
                'on_exit_shutdown': 'true',
            }.items(),
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='scratchpad_camera_bridge',
            arguments=[
                '/scratchpad/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                '/scratchpad/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            ],
            output='screen',
        ),
        node_heaps_spawner,
        node_cube_tracking,
        node_inspect_tag_corners,
    ])
