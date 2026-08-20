"""
Spawn cube heaps
"""

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    world_name = "warehouse_world"

    heaps_list = [
            {'x_pose': 4.0,
             'y_pose': 4.9,
             'mass': 0.1,
             'side_length': 0.1,
             'gap': 0.0005
             },
        ]

    CUBE_SDF = """
    <?xml version="1.0" ?>
    <sdf version="1.8">
        <model name="{model_name}">
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
    </sdf>
    """

    for heap in heaps_list:
            x_pose = heap['x_pose']
            y_pose = heap['y_pose']
            mass = heap['mass']
            side_length = heap['side_length']
            gap = heap['gap']
    
            inertia = mass * side_length**2 / 6
    
            cube_id = 1
            delay_step = 1.5
            delay = 3.0
    
            for x in range(2):
                for y in range(2):
                    for z in range(4):
    
                        if (x + y + z) < 3 or (x == 0 and y == 0):          # not very clever spawn logic
                            spawn_cube_node = Node(
                                 package='ros_gz_sim',
                                 executable='create',
                                 arguments=[
                                      '-world', world_name,
                                      'name', f'cube_{cube_id}',
                                      '-x', str(x_pose + x * (side_length + gap)),
                                      '-y', str(y_pose + y * (side_length + gap)),
                                      '-z', str((z + gap + 0.5) * side_length),
                                      '-string', CUBE_SDF.format(
                                           model_name=f'cube_{cube_id}', 
                                           mass=mass, 
                                           inertia=inertia, 
                                           side_length=side_length, 
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
