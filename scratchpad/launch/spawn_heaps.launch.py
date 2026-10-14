"""
Spawn cube heaps
"""

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from scratchpad.cube_geometry import generate_cube_tags
from scratchpad.cube_config import load_cube_settings

def generate_cube_sdf(model_name, mass, inertia, cube_side_length, offset, first_id):
    
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

    tags = generate_cube_tags(
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


         

def generate_launch_description():
    ld = LaunchDescription()

    world_name = "plane"

    settings = load_cube_settings()

    heaps_list = [
            {'x_pose': 4.0,
             'y_pose': 4.9,
             'mass': 0.1,
             'cube_side_length': settings['cube_side_length'],
             'gap': 0.001,
             'offset': settings['offset'],
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
                                           model_name=f'cube_{cube_id}', 
                                           mass=mass,
                                           inertia=inertia, 
                                           cube_side_length=cube_side_length,
                                           offset=offset,
                                           first_id=((cube_id - 1) * 24),
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
