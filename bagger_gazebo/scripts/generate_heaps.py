"""
TODO:   read heaps params from config file
TODO:   different geometries of possible heaps? + cleverer logic for heaps form generation
TODO:   colors in config
NOTE: Gazebo can only simulate apprx. 8 cubes with acceptable RTF in same heap
"""

import numpy as np

CUBE_SDF = """
<model name="{model_name}">
    <pose>{x} {y} {z} 0 0 0</pose>
    <link name="link">
        <inertial>
            <mass>{mass}</mass>
            <inertia><ixx>{inertia}</ixx><iyy>{inertia}</iyy><izz>{inertia}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <collision name="collision">
            <geometry><box><size>{side_length} {side_length} {side_length}</size></box></geometry>
        </collision>
        <visual name="visual">
            <geometry><box><size>{side_length} {side_length} {side_length}</size></box></geometry>
            <material>
                <ambient>1.0 0.0 0.0 1.0</ambient>
                <diffuse>1.0 0.0 0.0 1.0</diffuse>
            </material>
        </visual>
    </link>
</model>
"""

heaps_list = [
            {'x_pose': 4,
             'y_pose': 4,
             'mass': 0.1,
             'side_length': 0.1,
             'gap': 0.0005
             },
        ]

file_path = '../worlds/heaps.sdf'

def generate_sdf():
    sdf_content = '<?xml version="1.0" ?><sdf version="1.6"><model name="all_heaps">'
    
    for heap in heaps_list:
        x_pose = heap['x_pose']
        y_pose = heap['y_pose']
        mass = heap['mass']
        side_length = heap['side_length']
        gap = heap['gap']

        inertia = mass * side_length**2 / 6

        cube_id = 1

        for x in range(2):
            for y in range(2):
                for z in range(4):

                    if (x + y + z) < 3 or (x == 0 and y == 0):          # not very clever spawn logic
                        sdf_content += CUBE_SDF.format(
                            model_name=f'cube_{cube_id}', mass=mass, inertia=inertia, 
                            side_length=side_length, 
                            x=x_pose + x * (side_length + gap),
                            y=y_pose + y * (side_length + gap),
                            z=(z + gap + 0.5) * side_length,
                            )

                    cube_id+=1

        sdf_content += '</model></sdf>'

        with open(file_path, 'w') as f:
            f.write(sdf_content)

if __name__ == '__main__':
    generate_sdf()