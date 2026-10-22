# Bagger — ROS 2

## Demo

## Cube collection sequence overview
Bagger collects cubes and unloads them in a defined area.
The cube collection sequence consists of 7 steps:

### Wait for cube pose
Bagger receives the ground truth position of a chosen cube from Gazebo and saves for navigation.

### Navigate to cube
Nav2 navigates Bagger to the cube.

### Align arm to target heiht
After Bagger arrives at the cube, the camera detects AprilTags.
Each cube has its own board of tags.
This allows Bagger to distinguish between cubes, estimate the target cube’s position more precisely, and adjust the arm’s height to collect it.
The cube’s ground truth position would only be suitable if we also used Bagger’s ground truth position, because odometry cannot provide an exact position due to drift.
This is even more relevant for the next step.

### Approach and collect cube
Bagger rotates until the arm roughly points toward the cube.
Then it slowly approaches the cube with the bucket wheel rotating.
As soon as the bucket wheel’s collision mesh (which is smaller than its visual mesh) contacts a cube, the cube is teleported into the container.
The collection process is not precise enough, so additional cubes are often collected along with the target cube.
Collection is detected by monitoring weight changes in the container. As soon as the weight exceeds a threshold, Bagger stops driving and stops the bucket wheel.

### Back away from heap
Bagger backs away from the heap by the same distance it traveled while approaching it.

### Navigate to dumping are
Nav2 receives a goal to navigate to the dumping area.

### Dump container
Bagger slowly moves backward. When the rear sensors (two small red cubes behind the wheels) detect an increase in distance, Bagger immediately stops and executes the dumping sequence. Finally, it drives forward by the same distance it moved backward. The cube collection sequence then ends.

## Limits
The main limitation is that Bagger cannot autonomously discover cubes.
If Bagger cannot immediately detect the target cube after arriving, it stops.
Therefore, if the target cube is hidden behind other cubes or the bucket wheel blocks all its AprilTags, Bagger cannot detect, approach, or collect it.

Once Bagger has detected the cube during collection, it can use the position estimate, even if no tags are visible at the moment of collection.

## Installation

Requires Ubuntu 24.04, ROS 2 Kilted Desktop, Git, `colcon` and `rosdep`.

### Install the project
```bash
source /opt/ros/kilted/setup.bash
mkdir -p ~/bagger_ws/src
git clone https://github.com/TsikhanNizavets/bagger_ros.git ~/bagger_ws/src/bagger_ros
cd ~/bagger_ws
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
```

## Usage

The commands below use Bash. Keep the simulation running while using cube collection or teleop.

### Start the simulation
```bash
source ~/bagger_ws/install/setup.bash
ros2 launch bagger_bringup sim.launch.py
```

Optional launch arguments:

| Argument | Default | Description |
| --- | --- | --- |
| `spawn_heaps` | `true` | Spawn heaps of cubes. |
| `camera_debug_view` | `false` | Publish debug images with detected AprilTags highlighted. |
| `use_rviz` | `true` | Launch RViz. |
| `use_nav2` | `true` | Launch Nav2, which is required for autonomous cube collection. |

Append arguments to the launch command. For example, to show the camera debug view and disable RViz:

```bash
ros2 launch bagger_bringup sim.launch.py camera_debug_view:=true use_rviz:=false
```

> With `spawn_heaps:=false`, the cube tracker still starts and requires an existing cube board configuration file at `/tmp/cube_boards_3d.yaml` by default. If this file is missing, the tracker fails to start.


### Start cube collection

Start the simulation first and wait for the cubes to spawn and Nav2 to finish starting. Then run the following commands in another terminal:

```bash
source ~/bagger_ws/install/setup.bash
ros2 launch bagger_autonomy orchestrator.launch.py
```

The orchestrator currently targets `cube_2`.

### Start teleop (manual Bagger control)

With the simulation running, open another terminal:

```bash
source ~/bagger_ws/install/setup.bash
ros2 run bagger_teleop teleop_bagger --ros-args -p use_sim_time:=true
```

Keyboard controls are shown in the terminal.

## Configuration

Edit `bagger_gazebo/config/heaps_settings.yaml` to adjust heap positions, cube size, and cube mass. Add or remove heap entries to change the number of heaps.

Restart the simulation after changing the configuration.

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.

For credits and licenses of the included worlds, see [Third-party notices](THIRD_PARTY_NOTICES.md).