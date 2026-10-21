import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_name = 'bagger_navigation'

    pkg_share = get_package_share_directory(package_name)

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    map_yaml_file = LaunchConfiguration('map', default=os.path.join(pkg_share, 'maps', 'warehouse_map.yaml'))
    params_file = LaunchConfiguration('params_file', default=os.path.join(pkg_share, 'config', 'nav2_params.yaml'))
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    rviz_config_file = LaunchConfiguration('rviz_config', default=os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz'))
    use_rviz = LaunchConfiguration('use_rviz')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )

    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true',
            'keepout_mask': os.path.join(
                pkg_share, 'maps', 'warehouse_keepout_mask.yaml'
            ),
            'use_speed_zones': 'False',
        }.items(),
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')
        ),
        condition=IfCondition(use_rviz),
        launch_arguments={
            'namespace': '',
            'use_sim_time': use_sim_time,
            'rviz_config': rviz_config_file,
        }.items()
    )

    return LaunchDescription([
        declare_use_rviz_cmd,
        nav2_bringup_cmd,
        rviz_cmd,
    ])
