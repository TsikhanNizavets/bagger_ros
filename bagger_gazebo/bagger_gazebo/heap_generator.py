from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


class HeapGenerator():
    def __init__(self):
        self.heap_settings = self._load_settings()

        self.SPAWN_INTERVAL = 1.5
        self.INITIAL_SPAWN_DELAY = 10.0

    def _load_settings(self):
        package_directory = Path(
            get_package_share_directory('bagger_gazebo')
        )
        config_path = package_directory / 'config' / 'heaps_settings.yaml'

        with config_path.open() as file:
            return yaml.safe_load(file)

    def _generate_cube_sdf(self, model_name, mass, inertia, cube_side_length):
        
        CUBE_SDF_BEGIN = """
        <?xml version="1.0" ?>
        <sdf version="1.8">
            <model name="{model_name}">
                <link name="link">
                    <inertial>
                        <mass>{mass}</mass>
                        <inertia><ixx>{inertia}</ixx><iyy>{inertia}</iyy><izz>{inertia}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
                    </inertial>
                    <collision name="collision">
                        <geometry><box><size>{cube_side_length} {cube_side_length} {cube_side_length}</size></box></geometry>
                    </collision>
                    <visual name="cube">
                        <geometry><box><size>{cube_side_length} {cube_side_length} {cube_side_length}</size></box></geometry>
                        <material>
                            <ambient>1.0 0.0 0.0 1.0</ambient>
                            <diffuse>1.0 0.0 0.0 1.0</diffuse>
                        </material>
                    </visual>
                </link>

                <plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">
                    <publish_model_pose>true</publish_model_pose>
                    <publish_link_pose>true</publish_link_pose>
                    <use_pose_vector_msg>true</use_pose_vector_msg>
                    <update_frequency>1</update_frequency>
                    <topic>/ground_truth/cubes</topic>
                </plugin>

            </model>
        </sdf>
        """

        CUBE_SDF_BEGIN = CUBE_SDF_BEGIN.format(
            model_name=model_name,
            mass=mass,
            inertia=inertia,
            cube_side_length=cube_side_length
        )

        return CUBE_SDF_BEGIN

    def generate_launch_description(self, condition=None):
        ld = LaunchDescription()

        world_name = "warehouse_world"

        spawn_delay = self.INITIAL_SPAWN_DELAY

        cube_id = 0

        for heap_setting in self.heap_settings:
            x_pose = heap_setting['x_pose']
            y_pose = heap_setting['y_pose']
            mass = heap_setting['mass']
            cube_side_length = heap_setting['cube_side_length']
            gap = heap_setting['gap']

            inertia = mass * cube_side_length**2 / 6

            for x in range(2):
                for y in range(2):
                    for z in range(4):

                        if (x + y + z) < 3 or (x == 0 and y == 0):          # not very clever spawn logic
                            spawn_cube_node = Node(
                                    package='ros_gz_sim',
                                    executable='create',
                                    arguments=[
                                        '-world', world_name,
                                        '-name', f'cube_{cube_id}',
                                        '-x', str(x_pose + x * (cube_side_length + gap)),
                                        '-y', str(y_pose + y * (cube_side_length + gap)),
                                        '-z', str((z + gap + 0.5) * cube_side_length),
                                        '-string', self._generate_cube_sdf(
                                            model_name=f'cube_{cube_id}', 
                                            mass=mass,
                                            inertia=inertia, 
                                            cube_side_length=cube_side_length,
                                            ),
                                    ],
                                    output='screen',
                            )

                            delayed_spawn = TimerAction(
                                    period=spawn_delay,
                                    actions=[spawn_cube_node],
                                    condition=condition,
                            )

                            ld.add_action(delayed_spawn)

                            spawn_delay += self.SPAWN_INTERVAL    
                            cube_id+=1

        return ld
