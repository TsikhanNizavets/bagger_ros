"""
TODO:   potentially config file for spawn_heaps.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    package_name = 'bagger_gazebo'

    pkg_share = get_package_share_directory(package_name)
    pkg_prefix = get_package_prefix(package_name)

    description_prefix = get_package_prefix('bagger_description')

    resource_paths = f"{pkg_prefix}/share:{description_prefix}/share"

    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += f":{resource_paths}"
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = f"{resource_paths}"

    world_path = os.path.join(pkg_share, 'worlds', 'warehouse_world.sdf')

    heaps_file_arg = DeclareLaunchArgument(
        'heaps_file',
        default_value='heaps.sdf',
        description='SDF file containing the heaps',
    )

    declare_spawn_heaps_cmd = DeclareLaunchArgument(
        'spawn_heaps',
        default_value='true',
        description='Whether to spawn heaps',
    )

    declare_ore_spawner_cmd = DeclareLaunchArgument(
        'ore_spawner',
        default_value='false',
        description='Whether to ore spawner node (spheres)'
    )

    node_ore_spawner = Node(
        package='bagger_gazebo',
        executable='ore_spawner',
        condition=IfCondition(LaunchConfiguration('ore_spawner')),
        output='screen',
    )

    node_bucket_wheel_collector = Node(
        package='bagger_gazebo',
        executable='bucket_wheel_collector',
        output='screen'
    )

    node_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/TwistStamped@gz.msgs.Twist',
            '/arm_cmd@std_msgs/msg/Float64@gz.msgs.Double',
            '/bucket_wheel_cmd@std_msgs/msg/Float64@gz.msgs.Double',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/container_cmd@std_msgs/msg/Float64]gz.msgs.Double',                
            '/wheel_clearance/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/world/warehouse_world/create@ros_gz_interfaces/srv/SpawnEntity',
            '/world/warehouse_world/set_pose@ros_gz_interfaces/srv/SetEntityPose',
            '/model/bagger/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/right_drop_off/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/left_drop_off/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/world/warehouse_world/model/bagger/link/bucket_wheel/sensor/bucket_wheel_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
        ],
        remappings=[
            ('/model/bagger/pose', '/ground_truth/pose'),
            ('/world/warehouse_world/model/bagger/link/bucket_wheel/sensor/bucket_wheel_contact/contact', '/bucket_wheel/contacts'),
        ],
        output='screen',
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    node_heaps_spawner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'spawn_heaps.launch.py')),
        condition=IfCondition(LaunchConfiguration('spawn_heaps')),
    )

    return LaunchDescription([
        heaps_file_arg,
        declare_spawn_heaps_cmd,
        declare_ore_spawner_cmd,
        node_ore_spawner,
        node_ros_gz_bridge,
        node_heaps_spawner,
        node_bucket_wheel_collector,
        gazebo,
    ])
