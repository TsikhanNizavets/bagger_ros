from pathlib import Path

import cv2
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


class TaggedHeapGenerator():
    def __init__(self, cube_boards_path):
        self.cube_boards_path = Path(cube_boards_path)

        self.heap_settings = self._load_settings()

        self.SPAWN_INTERVAL = 1.5
        self.INITIAL_SPAWN_DELAY = 10.0

        self.boards = []

    def _load_settings(self):
        package_directory = Path(
            get_package_share_directory('bagger_gazebo')
        )
        config_path = package_directory / 'config' / 'heaps_settings.yaml'

        with config_path.open() as file:
            return yaml.safe_load(file)

    def generate_cube_tag_geometry(self, cube_side_length, offset=0.0002, first_id=0):
        texture_size = cube_side_length / 5
        marker_size = 0.8 * texture_size

        marker_corners = np.array([
            [-marker_size / 2,  marker_size / 2, 0],
            [ marker_size / 2,  marker_size / 2, 0],
            [ marker_size / 2, -marker_size / 2, 0],
            [-marker_size / 2, -marker_size / 2, 0],
        ], dtype=np.float32)

        face_totations = [
            (np.pi / 2, 0, np.pi / 2),
            (np.pi / 2, 0, -np.pi / 2),
            (np.pi / 2, 0, np.pi),
            (np.pi / 2, 0, 0),
            (0, 0, 0),
            (0, np.pi, 0),
        ]

        tags = []

        for face_index, rpy in enumerate(face_totations):
            rotation = Rotation.from_euler('xyz', rpy)

            for row in range(2):
                for column in range(2):
                    local_position = [
                        0.4 * cube_side_length * (2 * column - 1),
                        0.4 * cube_side_length * (1 - 2 * row),
                        cube_side_length / 2 + offset,
                    ]

                    position = rotation.apply(local_position)
                    corners = rotation.apply(marker_corners) + position

                    tags.append({
                        'id': first_id + 4 * face_index + 2 * row + column,
                        'position': position,
                        'rpy': rpy,
                        'texture_size': texture_size,
                        'corners': corners,
                    })

        return tags

    def _add_cube_board(self, cube_id, cube_side_length, offset):
        tags = self.generate_cube_tag_geometry(
            cube_side_length,
            offset=offset,
            first_id=cube_id * 24,
        )

        markers = []

        for tag in tags:
            markers.append({
                'id': tag['id'],
                'corners': tag['corners'].round(9).tolist(),
            })

        self.boards.append({
            'name': f'cube_{cube_id}',
            'markers': markers,
        })

    def _write_cube_boards_yaml(self):
        with self.cube_boards_path.open('w') as file:
            yaml.safe_dump(self.boards, file, sort_keys=False)

    @staticmethod
    def generate_textures():
        dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_APRILTAG_36h11
        )

        output_directory = Path(__file__).resolve().parents[1] / 'textures'
        output_directory.mkdir(parents=True, exist_ok=True)

        number_of_tags = 587
        marker_pixels = 200
        margin_pixels =  25

        for tag_id in range(number_of_tags):
            marker = cv2.aruco.drawMarker(
                dictionary,
                tag_id,
                marker_pixels,
            )

            texture = cv2.copyMakeBorder(
                marker,
                margin_pixels, margin_pixels,
                margin_pixels, margin_pixels,
                cv2.BORDER_CONSTANT,
                value=255,
            )

            filename = output_directory / f'tag36_11_{tag_id:05d}.png'

            if not cv2.imwrite(str(filename), texture):
                raise RuntimeError(f'Could not save {filename}')

        print(f'Generated {number_of_tags} textures in {output_directory}')

    def _generate_cube_sdf(self, model_name, mass, inertia, cube_side_length, offset, first_id):
        
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
        """

        CUBE_SDF_TAG = """
                    <visual name="apriltag_{tag_num}">
                        <pose>{tag_x} {tag_y} {tag_z} {rot_x} {rot_y} {rot_z}</pose>
                        <geometry>
                            <plane>
                                <normal>0 0 1</normal>
                                <size>{tag_size}</size>
                            </plane>
                        </geometry>
                        <material>
                            <ambient>1.0 1.0 1.0 1.0</ambient>
                            <diffuse>1.0 1.0 1.0 1.0</diffuse>
                            <emissive>0.2 0.2 0.2 1.0</emissive>
                            <pbr>
                                <metal>
                                    <albedo_map>package://bagger_gazebo/textures/tag36_11_{tag_num:05d}.png</albedo_map>
                                    <emissive_map>package://bagger_gazebo/textures/tag36_11_{tag_num:05d}.png</emissive_map>
                                    <roughness>1.0</roughness>
                                    <metalness>0.0</metalness>
                                </metal>
                            </pbr>
                        </material>
                    </visual>
        """

        CUBE_SDF_END = """
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
        
        CUBE_SDF_MIDDLE = ""

        tags = self.generate_cube_tag_geometry(
            cube_side_length,
            offset=offset,
            first_id=first_id,
        )

        for tag in tags:
            x, y, z = tag['position']
            roll, pitch, yaw = tag['rpy']
            size = tag['texture_size']

            CUBE_SDF_MIDDLE += CUBE_SDF_TAG.format(
                tag_num=tag['id'],
                tag_x=x,
                tag_y=y,
                tag_z=z,
                rot_x=roll,
                rot_y=pitch,
                rot_z=yaw,
                tag_size=f"{size} {size}",
            )

        return CUBE_SDF_BEGIN + "\n" + CUBE_SDF_MIDDLE + "\n" + CUBE_SDF_END

    def generate_launch_description(self):
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
            offset=heap_setting['offset']

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
                                            offset=offset,
                                            first_id=cube_id * 24,
                                            ),
                                    ],
                                    output='screen',
                            )

                            delayed_spawn = TimerAction(
                                    period=spawn_delay,
                                    actions=[spawn_cube_node],
                            )

                            ld.add_action(delayed_spawn)

                            self._add_cube_board(cube_id, cube_side_length, offset)

                            spawn_delay += self.SPAWN_INTERVAL    
                            cube_id+=1

        self._write_cube_boards_yaml()

        return ld
