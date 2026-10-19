"""
TODO:   potentially config file for spawn_heaps.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_prefix

from bagger_gazebo.tagged_heap_generator import TaggedHeapGenerator


def create_heaps(context):
    cube_boards_path = LaunchConfiguration('cube_boards_path').perform(context)

    heap_generator = TaggedHeapGenerator(cube_boards_path)

    return [heap_generator.generate_launch_description()]

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

    xacro_file = PathJoinSubstitution([
        FindPackageShare('bagger_gazebo'),
        'urdf',
        'bagger_sim.urdf.xacro'
    ])

    robot_description_content = Command(
        [PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', xacro_file]
    )
    robot_description = ParameterValue(robot_description_content, value_type=str)

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

    declare_cube_boards_path = DeclareLaunchArgument(
        'cube_boards_path',
        default_value='/tmp/cube_boards_3d.yaml',
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

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }]
    )

    node_spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'bagger',
            '-x', '13.5',
            '-y', '0.0',
            '-z', '0.05',
            '-Y', '3.1428',
        ]
    )

    node_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/TwistStamped@gz.msgs.Twist',
            '/arm_cmd@std_msgs/msg/Float64@gz.msgs.Double',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/container_cmd@std_msgs/msg/Float64]gz.msgs.Double',                
            '/wheel_clearance/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/world/warehouse_world/create@ros_gz_interfaces/srv/SpawnEntity',
            '/world/warehouse_world/set_pose@ros_gz_interfaces/srv/SetEntityPose',
            '/bagger/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/right_drop_off/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/left_drop_off/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/bucket_wheel/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/ground_truth/cubes@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/container/wrench@geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench',
        ],
        remappings=[
            ('/bagger/pose', '/ground_truth/pose'), # rename to /ground_truth/bagger (like ground_truth/cubes)
        ],
        output='screen',
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )
    
    node_heaps_spawner = OpaqueFunction(
        function=create_heaps,
        condition=IfCondition(LaunchConfiguration('spawn_heaps')),
    )

    return LaunchDescription([
        heaps_file_arg,
        declare_spawn_heaps_cmd,
        declare_ore_spawner_cmd,
        declare_cube_boards_path,
        node_robot_state_publisher,
        node_ore_spawner,
        node_ros_gz_bridge,
        node_heaps_spawner,
        node_bucket_wheel_collector,
        node_spawn_robot,
        gazebo,
    ])
