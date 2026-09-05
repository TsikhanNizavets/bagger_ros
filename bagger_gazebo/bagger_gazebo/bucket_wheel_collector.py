"""
Collect spawned heap cubes, if bucket wheel collides with them (minig)
"""

import random
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
from ros_gz_interfaces.msg import Contacts, Entity
from ros_gz_interfaces.srv import SetEntityPose
from geometry_msgs.msg import Pose
from tf2_msgs.msg import TFMessage


class BucketWheelCollector(Node):
    def __init__(self):
        super().__init__('bucket_wheel_collector')

        self.world_name = 'warehouse_world'
        self.target_prefix = 'cube'
        self.required_contact_duration = 1.5

        self.bucket_wheel_speed = 0.0
        self.bucket_wheel_min_speed = 0.1

        self.model_frame = 'bagger'
        self.ore_collection_point_frame = 'bagger/ore_collection_point'

        self.ore_collection_point_gt_transform = None
        self.model_gt_pose = None

        self.contact_timestamps = {}

        self.sub_gt_pose = self.create_subscription(TFMessage, '/ground_truth/pose', self.gt_pose_callback, 10)
        self.sub_contacts = self.create_subscription(Contacts, '/bucket_wheel/contacts', self.contact_callback, 10)
        self.sub_joint_states = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.teleport_client = self.create_client(SetEntityPose, '/world/warehouse_world/set_pose')

    def contact_callback(self, msg):
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        active_contacts = set()

        if abs(self.bucket_wheel_speed) > self.bucket_wheel_min_speed:
            for contact in msg.contacts:
                obj_a = contact.collision1.name
                obj_b = contact.collision2.name

                cube_name = self.extract_cube_name(obj_a, obj_b)
                if not cube_name:
                    continue

                active_contacts.add(cube_name)

                if cube_name not in self.contact_timestamps:
                    self.contact_timestamps[cube_name] = current_time
                else:
                    eslapsed = current_time - self.contact_timestamps[cube_name]
                    if eslapsed >= self.required_contact_duration:
                        self.collect_cube(cube_name)
                        del self.contact_timestamps[cube_name]

        for cube_name in list(self.contact_timestamps.keys()):
            if cube_name not in active_contacts:
                del self.contact_timestamps[cube_name]

    def collect_cube(self, cube_name):
        if not self.teleport_client.service_is_ready():
            return

        req = SetEntityPose.Request()
        req.entity.name = cube_name
        req.entity.type = Entity.MODEL
        req.pose = Pose()

        x, y, z = self.get_ore_collection_point_coordinates()

        if x is None:
            self.get_logger().warn('Skipping spawn: spawn_ore_point frame not found.')
            return

        req.pose.position.x = x
        req.pose.position.y = y
        req.pose.position.z = z

        future = self.teleport_client.call_async(req)

    def extract_cube_name(self, name_a, name_b):
        for name in (name_a, name_b):
            if self.target_prefix in name:
                return name.split('::')[0]
        return None

    def get_ore_collection_point_coordinates(self):
        if self.model_gt_pose is None or self.ore_collection_point_gt_transform is None:
            self.get_logger().warn('Ground truth transform not received yet.')
            return None, None, None

        local_x = self.ore_collection_point_gt_transform.transform.translation.x + random.uniform(-0.1, 0.1)
        local_y = self.ore_collection_point_gt_transform.transform.translation.y + random.uniform(-0.05, 0.05)
        local_z = self.ore_collection_point_gt_transform.transform.translation.z

        gt_pos = self.model_gt_pose.transform.translation
        gt_ori = self.model_gt_pose.transform.rotation

        r = R.from_quat([gt_ori.x, gt_ori.y, gt_ori.z, gt_ori.w])
        world_offset = r.apply([local_x, local_y, local_z])

        gt_x = gt_pos.x + world_offset[0]
        gt_y = gt_pos.y + world_offset[1]
        gt_z = gt_pos.z + world_offset[2]

        return gt_x, gt_y, gt_z

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

    def joint_state_callback(self, msg):
        if 'bucket_wheel_joint' in msg.name:
            idx = msg.name.index('bucket_wheel_joint')
            self.bucket_wheel_speed = msg.velocity[idx]

def main(args=None):
    rclpy.init(args=args)
    node = BucketWheelCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
