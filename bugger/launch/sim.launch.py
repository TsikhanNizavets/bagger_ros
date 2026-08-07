import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    package_name = 'bugger'

    pkg_share = get_package_share_directory(package_name)
    pkg_prefix = get_package_prefix(package_name)

    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += f":{pkg_prefix}/share"
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = f":{pkg_prefix}/share"

    urdf_path = os.path.join(pkg_share, 'urdf', 'bugger.urdf')
    with open(urdf_path, 'r') as file:
        robot_description_config = file.read()

    world_path = os.path.join(pkg_share, 'worlds', 'warehouse_world.sdf')

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    map_yaml_file = LaunchConfiguration('map', default=os.path.join(pkg_share, 'maps', 'warehouse_map.yaml'))
    params_file = LaunchConfiguration('params_file', default=os.path.join(pkg_share, 'config', 'nav2_params.yaml'))
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    rviz_config_file = LaunchConfiguration('rviz_config', default=os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz'))

    twist_mux_config = os.path.join(pkg_share, 'config', 'twist_mux.yaml')

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config, 'use_sim_time': True}],
    )

    container_unloader_node = Node(             
        package='bugger',
        executable='container_unloader',
        name='container_unloader',
        output='screen',
    )

    node_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/TwistStamped@gz.msgs.Twist',
            '/arm_cmd@std_msgs/msg/Float64@gz.msgs.Double',
            '/wheel_cmd@std_msgs/msg/Float64@gz.msgs.Double',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/container_cmd@std_msgs/msg/Float64]gz.msgs.Double',                
            '/arm_clearance/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        output='screen',
    )

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_config],
        remappings=[('/cmd_vel_out', '/cmd_vel')],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true',
        }.items(),
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')
        ),
        launch_arguments={
            'namespace': '',
            'use_sim_time': use_sim_time,
            'rviz_config': rviz_config_file,
        }.items()
    )

    return LaunchDescription([
        node_robot_state_publisher,
        container_unloader_node,
        node_ros_gz_bridge,
        twist_mux_node,
        gazebo,
        bringup_cmd,
        rviz_cmd,
    ])
