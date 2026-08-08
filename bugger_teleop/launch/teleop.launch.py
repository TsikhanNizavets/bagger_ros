import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    manipulation_dir = get_package_share_directory('bugger_manipulation')

    bugger_teleop_node = Node(
        package='bugger_teleop',
        executable='teleop_bugger',
        name='teleop_bugger',
        output='screen',
    )

    # doesn't work
    '''container_unloader_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(manipulation_dir, 'launch', 'container_unloader.launch.py')),
    )'''

    return LaunchDescription([
        bugger_teleop_node,
        # container_unloader_launch,
    ])
