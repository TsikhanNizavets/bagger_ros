import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('bagger_perception'),
        'config',
    )

    return LaunchDescription([
        Node(
            package='aruco_opencv',
            executable='aruco_tracker_autostart',
            namespace='bagger_perception',
            name='aruco_tracker',
            parameters=[
                os.path.join(config_dir, 'cube_tracker.yaml'),
                {
                    'board_descriptions_path': os.path.join(
                        config_dir, 'cube_board.yaml'
                    ),
                },
            ],
            output='screen',
        ),
    ])