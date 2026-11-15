# Bagger — ROS 2

## Demo

https://github.com/user-attachments/assets/bec8a56f-f46e-4575-90fa-11d33ccc4e64

## Cube collection sequence overview
Bagger collects cubes and unloads them in a defined area.
The cube collection task consists of 6 steps:

### Get approximate target position
Bagger receives the ground truth position of a random cube from Gazebo and saves it for navigation.

### Navigate to target
Nav2 navigates Bagger to the cube.

### Attemp or recovery
#### Attemp
- #### Align arm to target height
  After Bagger arrives at the cube, the YOLOE11 model tries to detect red cubes.
  The YOLOE11 model generates a mask on the color image for detected cubes.
  This mask is applied to the depth image.
  The mean position of all points in the masked depth image is calculated and assumed to be the target’s position.
  
  From the detected cubes, Bagger chooses the closest one within a threshold range and adjusts the arm’s height accordingly.
  This approach allows Bagger to precisely estimate the target cube’s relative position and collect it.
  If there are no cubes in range, the behaviour reports failure, which is handled in the recovery step.
  
  The cube’s ground truth position would only be suitable if we also used Bagger’s ground truth position, because localisation cannot provide an exact position due to drift, which causes big errors in relative position.
  This is even more relevant for the next step.

- #### Approach and collect target
  Bagger rotates until the arm roughly points toward the cube.
  Then it slowly approaches the cube with the bucket wheel rotating.
  As soon as the bucket wheel’s collision mesh (which is smaller than its visual mesh) contacts a cube, the cube is teleported into the container.
  The collection process is not precise enough, so additional cubes are often collected along with the target cube.
  Collection is detected by monitoring weight changes in the container. Once a stable change in the container’s estimated load exceeds a threshold, Bagger stops driving and stops the bucket wheel.

#### Recovery
- #### Failre recovery
  It might happen that, at the current position, the YOLOE11 model fails to detect any cubes, even though they are clearly visible in the image, or Bagger misses cubes because they fall down or their estimated positions are imprecise.
  Then Bagger starts slowly moving back. After that, the attempt runs again.
  If no cubes are visible yet, recovery repeats.
  This sequence repeats until YOLOE11 detects a cube and Bagger can perform collection.

### Back away from heap
Bagger backs away from the heap by the same distance it traveled while approaching it.

### Navigate to dumping area
Nav2 receives a goal to navigate to the dumping area.

### Dump container
Bagger slowly moves backward.
When the rear sensors (two small red cubes behind the wheels) detect an increase in distance, Bagger immediately stops and unloads the container.
Finally, it drives forward by the same distance it moved backward.
The cube collection sequence then ends.

## Limitations
The main limitation is that Bagger is programmed to collect only one cube at a time and is unable to discover cubes autonomously.
It needs the ground truth position from Gazebo to know where to drive to.

Additionally, Bagger does avoid collisions with cubes.

## Installation

Requires Ubuntu 24.04, ROS 2 Kilted Desktop, Python 3.12,
Git, colcon, rosdep and python3-venv.

### Clone the repositories and install ROS dependencies

```bash
source /opt/ros/kilted/setup.bash
mkdir -p ~/bagger_ws/src

git clone --branch main https://github.com/TsikhanNizavets/bagger_ros.git ~/bagger_ws/src/bagger_ros

git clone --branch main https://github.com/TsikhanNizavets/bagger_perception.git ~/bagger_ws/src/bagger_perception

cd ~/bagger_ws
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro kilted --skip-keys python3-ultralytics-pip -y
```

Ultralytics is installed separately inside the Python environment below.

> NOTE: [bagger_perception](https://github.com/TsikhanNizavets/bagger_perception) is a separate repository containing the perception packages required for cube detection.

### Create the Python environment

```bash
/usr/bin/python3 -m venv --system-site-packages .venv
source .venv/bin/activate
```

### Install PyTorch

Inside the virtual environment, install matching PyTorch and torchvision versions for your hardware by following the [official installation instructions](https://pytorch.org/get-started/locally/).

The development environment uses PyTorch `2.14.1` and torchvision `0.29.1` with CUDA 12.6. Select an appropriate CPU or GPU build for your machine.

### Install the remaining Python dependencies

These versions match the development environment:

```bash
python -m pip install \
  "setuptools==79.0.1" \
  "numpy==1.26.4" \
  "opencv-python==4.11.0.86" \
  "ultralytics==8.4.192" \
```

### Build the workspace

```bash
python "$(command -v colcon)" build --symlink-install
```

## Usage

### Start the simulation
```bash
source ~/bagger_ws/install/setup.bash
ros2 launch bagger_bringup sim.launch.py
```

The first YOLOE startup requires internet access to download
`yoloe-11s-seg.pt` and `mobileclip_blt.ts` if they are missing.

Optional launch arguments:

| Argument | Default | Description |
| --- | --- | --- |
| `spawn_heaps` | `true` | Spawn heaps of cubes. |
| `use_rviz` | `true` | Launch RViz. |
| `use_nav2` | `true` | Launch Nav2, which is required for autonomous cube collection. |
| `show_yoloe_detections` | `false` | Show YOLOE11's view with detections. |
| `show_point_cloud` | `false` | Show targets' point clouds. |

Append arguments to the launch command. For example, to publish the camera debug images and disable RViz:

```bash
ros2 launch bagger_bringup sim.launch.py show_yoloe_detections:=true use_rviz:=false
```

### Start cube collection

Start the simulation first and wait for the cubes to spawn and Nav2 to finish starting. Then run the following commands in another terminal:

```bash
source ~/bagger_ws/install/setup.bash
ros2 launch bagger_autonomy orchestrator.launch.py
```

### Start teleop (manual Bagger control)

With the simulation running, open another terminal:

```bash
source ~/bagger_ws/install/setup.bash
ros2 run bagger_teleop teleop_bagger
```

Keyboard controls are shown in the terminal.

## Configuration

Edit `bagger_gazebo/config/heaps_settings.yaml` to adjust heap positions, cube size, and cube mass. Add or remove heap entries to change the number of heaps.

Restart the simulation after changing the configuration.

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.

For credits and licenses of the included worlds, see [Third-party notices](THIRD_PARTY_NOTICES.md).
