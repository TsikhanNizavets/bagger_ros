"""
Spawn cube heaps
"""

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_cube_sdf(model_name, mass, inertia, cube_side_length, offset, tag_size, first_tag_num):
    
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
                    <pose>{tag_x} {tag_y} {tag_z} -1.5708 0 0</pose>
                    <geometry>
                        <plane>
                            <normal>{tag_normal}</normal>
                            <size>{tag_size}</size>
                        </plane>
                    </geometry>
                    <material>
                        <ambient>1.0 1.0 1.0 1.0</ambient>
                        <diffuse>1.0 1.0 1.0 1.0</diffuse>
                        <emissive>0.2 0.2 0.2 1.0</emissive>
                        <pbr>
                            <metal>
                                <albedo_map>package://scratchpad/textures/tag36_11_{tag_num:05d}.png</albedo_map>
                                <emissive_map>package://scratchpad/textures/tag36_11_{tag_num:05d}.png</emissive_map>
                                <roughness>1.0</roughness>
                                <metalness>0.0</metalness>
                            </metal>
                        </pbr>
                    </material>
                </visual>
    """

    CUBE_SDF_END = """
            </link>
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
    tag_normal = ["1 0 0", "-1 0 0", "0 1 0", "0 -1 0", "0 0 1", "0 0 -1"]

    for i in range(6):
        for j in range(2):
            for k in range(2):
                CUBE_SDF_MIDDLE = CUBE_SDF_MIDDLE + "\n" + CUBE_SDF_TAG.format(
                    tag_num=(first_tag_num + 4 * i + 2 * j + k),
                    tag_x=(cube_side_length/2 + offset) * (1 - 2 * i) if i in [0, 1] else 2 * cube_side_length / 5 * (1 - 2 * (j if i in [2, 3] else k)),
                    tag_y=(cube_side_length/2 + offset) * (1 - 2 * (i - 2)) if i in [2, 3] else 2 * cube_side_length / 5 * (-1 + 2 * k),
                    tag_z=(cube_side_length/2 + offset) * (1 - 2 * (i - 4)) if i in [4, 5] else 2 * cube_side_length / 5 * (1 - 2 * j),
                    tag_size=tag_size,
                    tag_normal=tag_normal[i]
                )

    return CUBE_SDF_BEGIN + "\n" + CUBE_SDF_MIDDLE + "\n" + CUBE_SDF_END


         

def generate_launch_description():
    ld = LaunchDescription()

    world_name = "plane"

    heaps_list = [
            {'x_pose': 4.0,
             'y_pose': 4.9,
             'mass': 0.1,
             'cube_side_length': 0.1,
             'gap': 0.001,
             'offset': 0.0002,
             },
        ]

    for heap in heaps_list:
            x_pose = heap['x_pose']
            y_pose = heap['y_pose']
            mass = heap['mass']
            cube_side_length = heap['cube_side_length']
            gap = heap['gap']
            offset=heap['offset']
    
            inertia = mass * cube_side_length**2 / 6
    
            cube_id = 1
            delay_step = 1.5
            delay = 10.0
    
            for x in range(1):
                for y in range(1):
                    for z in range(1):
    
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
                                      '-string', generate_cube_sdf(
                                           model_name='cube_{cube_id}', 
                                           mass=mass,
                                           inertia=inertia, 
                                           cube_side_length=cube_side_length,
                                           offset=offset,
                                           tag_size=f"{cube_side_length/5} {cube_side_length/5}",
                                           first_tag_num=((cube_id - 1) * 24),
                                           ),
                                 ],
                                 output='screen',
                            )

                            delayed_spawn = TimerAction(
                                 period=delay,
                                 actions=[spawn_cube_node],
                            )

                            ld.add_action(delayed_spawn)

                            delay += delay_step    
                            cube_id+=1
    return ld
