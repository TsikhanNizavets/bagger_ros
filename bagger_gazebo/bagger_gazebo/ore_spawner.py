"""
"""

import math
import rclpy
import tf2_ros
from rclpy.time import Time
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, JointState
from sensor_msgs_py import point_cloud2
from ros_gz_interfaces.srv import SpawnEntity, SetEntityPose
from ros_gz_interfaces.msg import Entity
from geometry_msgs.msg import Pose
from tf2_msgs.msg import TFMessage
from scipy.spatial.transform import Rotation as R

SPHERE_SDF = """
<sdf version="1.6">
  <model name="{model_name}">
    <pose>0 0 0 0 0 0</pose>
    <link name="link">
      <inertial>
        <mass>0.1</mass>
        <inertia><ixx>0.0001</ixx><iyy>0.0001</iyy><izz>0.0001</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <collision name="collision">
        <geometry><sphere><radius>0.05</radius></sphere></geometry>
      </collision>
      <visual name="visual">
        <geometry><sphere><radius>0.05</radius></sphere></geometry>
        <material><ambient>0.4 0.25 0.1 1</ambient></material>
      </visual>
    </link>
  </model>
</sdf>
"""
                                                                        
class OreSpawner(Node):
    def __init__(self):
        super().__init__('ore_spawner')

        self.world_name = 'warehouse_world'
        self.pool_size = 5
        self.pool_index = 0
        self.pool_initialized = False

        self.model_frame = 'bagger'
        self.ore_collection_point_frame = 'bagger/ore_collection_point'

        self.spawn_client = self.create_client(SpawnEntity, f'/world/{self.world_name}/create')
        self.pose_client = self.create_client(SetEntityPose, f'/world/{self.world_name}/set_pose')

        self.sub_joint_states = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.sub_sensor = self.create_subscription(PointCloud2, '/wheel_clearance/points', self.sensor_callback, 10)

        self.model_gt_pose = None
        self.ore_collection_point_gt_transform = None               # Ground Truth Position for Ore Collection Point
        self.sub_gt_pose = self.create_subscription(TFMessage, '/ground_truth/pose', self.gt_pose_callback, 10)

        self.wheel_speed = 0.0
        self.min_distance = float('inf')
        self.mining_threshold_dist = 0.12
        self.wheel_min_speed = 0.5

        self._ore_pool_x = 18.0
        self._ore_pool_y = 0.0
        self._ore_pool_z = 10.0

        self.timer = self.create_timer(5, self.spawning_loop)
        self.get_logger().info('Ore spawner node initialized.')

    def try_init_pool(self):
        if not self.spawn_client.service_is_ready():
            self.get_logger().warn(
                f'Waiting for {self.spawn_client.srv_name}... Check service type with: '
                f'ros2 service type {self.spawn_client.srv_name}'
            )
            return False

        self.get_logger().info('Pre-spawning ore pool underground...')
        for i in range(self.pool_size):
            req = SpawnEntity.Request()
            req.entity_factory.name = f'ore_sphere_{i}'
            req.entity_factory.sdf = SPHERE_SDF.format(model_name=f'ore_sphere_{i}')
            req.entity_factory.pose.position.x = self._ore_pool_x
            req.entity_factory.pose.position.y = self._ore_pool_y
            req.entity_factory.pose.position.z = self._ore_pool_z
            self.spawn_client.call_async(req)

        self.pool_initialized = True
        self.get_logger().info('Ore pool successfully initialized.')
        return True

    def joint_state_callback(self, msg):
        if 'bucket_wheel_joint' in msg.name:
            idx = msg.name.index('bucket_wheel_joint')
            self.wheel_speed = msg.velocity[idx]

    def sensor_callback(self, msg):
        gen = point_cloud2.read_points(msg, field_names=['x', 'y', 'z'], skip_nans=True)

        distances = [math.sqrt(p[0]**2 + p[1]**2 + p[2]**2) for p in gen]
        self.min_distance = min(distances) if distances else float('inf')

    def spawning_loop(self):
        if not self.pool_initialized:
            if not self.try_init_pool():
                return

        if abs(self.wheel_speed) > self.wheel_min_speed and self.min_distance < self.mining_threshold_dist:
            self.teleport_ore()

    def teleport_ore(self):
        if not self.pose_client.service_is_ready():
            self.get_logger().warn(f'SetPose service {self.pose_client.srv_name} not ready.')
            return

        entity_name = f'ore_sphere_{self.pool_index}'
        self.pool_index = (self.pool_index + 1) % self.pool_size

        req = SetEntityPose.Request()
        req.entity = Entity()
        req.entity.name = entity_name
        req.pose = Pose()
        
        x, y, z = self.get_container_coordinates()

        if x is None:
            self.get_logger().warn('Skipping spawn: ore_collection_point frame not found.')
            return

        req.pose.position.x = x
        req.pose.position.y = y
        req.pose.position.z = z

        self.pose_client.call_async(req)

    def get_container_coordinates(self):
        if self.model_gt_pose is None or self.ore_collection_point_gt_transform is None:
            self.get_logger().warn('Ground truth position not received yet.')
            return None, None, None

        local_x = self.ore_collection_point_gt_transform.transform.translation.x
        local_y = self.ore_collection_point_gt_transform.transform.translation.y
        local_z = self.ore_collection_point_gt_transform.transform.translation.z

        gt_pos = self.model_gt_pose.transform.translation
        gt_ori = self.model_gt_pose.transform.rotation

        r = R.from_quat([gt_ori.x, gt_ori.y, gt_ori.z, gt_ori.w])
        world_offset = r.apply([local_x, local_y, local_z])

        gz_x = gt_pos.x + world_offset[0]
        gz_y = gt_pos.y + world_offset[1]
        gz_z = gt_pos.z + world_offset[2]

        return gz_x, gz_y, gz_z

    def gt_pose_callback(self, msg):
        count = 0
        for transform in msg.transforms:
            if transform.child_frame_id == self.ore_collection_point_frame:
                self.ore_collection_point_gt_transform = transform
                count += 1
            elif transform.child_frame_id == self.model_frame:
                self.model_gt_pose = transform
                count += 1
            elif count == 2:
                break

def main(args=None):
    rclpy.init(args=args)
    node = OreSpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()