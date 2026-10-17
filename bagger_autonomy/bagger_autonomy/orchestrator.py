import math
import time
from typing import Any

import py_trees
import py_trees_ros
import rclpy
import tf2_geometry_msgs
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped, Vector3, WrenchStamped
from nav2_msgs.action import NavigateToPose
from py_trees.common import Status
from rclpy.action import ActionClient
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Bool, Float64
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.time import Time
from control_msgs.msg import MultiDOFCommand


class WaitForCubePose(py_trees.behaviour.Behaviour):
    def __init__(self, name="Wait for cube pose"):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key("target_cube_name", access=py_trees.common.Access.READ)
        self.blackboard.register_key("target_cube_position", access=py_trees.common.Access.WRITE)

        self.blackboard.target_cube_position = None
        self.deadline = None

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.sub_cube_poses = self.node.create_subscription(TFMessage, '/ground_truth/cubes', self.cube_poses_callback, 10)


    def initialise(self):
        self.deadline = time.monotonic() + 60.0

    def update(self):
        if self.blackboard.target_cube_position is not None:
            return py_trees.common.Status.SUCCESS
        elif time.monotonic() >= self.deadline:
            self.node.get_logger().error('Time out waiting for target cube pose')
            return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.RUNNING

    def cube_poses_callback(self, msg):
        for transform in msg.transforms:
            if transform.child_frame_id == self.blackboard.target_cube_name and self.blackboard.target_cube_position is None:
                self.blackboard.target_cube_position = transform.transform.translation
                return


class NavigateToCube(py_trees.behaviour.Behaviour):
    def __init__(self, name="Navigate to cube"):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key("target_cube_name", access=py_trees.common.Access.READ)
        self.blackboard.register_key("target_cube_position", access=py_trees.common.Access.READ)
        
        self.goal_sent = False
        self.goal_done = False
        self.goal_success = False

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.nav_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

    def initialise(self):
        self.goal_sent = False
        self.goal_done = False
        self.goal_success = False
        self.goal_handle = None

    def update(self):
        if not self.goal_sent:
            try:
                trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                rx = trans.transform.translation.x
                ry = trans.transform.translation.y
            except Exception as e:
                self.node.get_logger().warn(f"Waiting for TF: {e}")
                return py_trees.common.Status.RUNNING # Keep trying

            cx = self.blackboard.target_cube_position.x
            cy = self.blackboard.target_cube_position.y
            angle_to_cube = math.atan2(cy - ry, cx - rx)
            
            pre_dock_x = cx - 0.90 * math.cos(angle_to_cube)        # check if navigation works for cubes in other places on map
            pre_dock_y = cy - 0.90 * math.sin(angle_to_cube)        # check if navigation works works for other initial bagger poses

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()

            goal_msg.pose.pose.position.x = pre_dock_x
            goal_msg.pose.pose.position.y = pre_dock_y
            
            goal_msg.pose.pose.orientation.z = math.sin(angle_to_cube / 2.0)
            goal_msg.pose.pose.orientation.w = math.cos(angle_to_cube / 2.0)

            self.logger.info('Before waiting')
            if not self.nav_client.server_is_ready():
                self.node.get_logger().warn("Waiting for Nav2 action server...")
                return py_trees.common.Status.RUNNING

            send_goal_future = self.nav_client.send_goal_async(goal_msg)
            send_goal_future.add_done_callback(self.goal_response_callback)
            
            self.goal_sent = True
            self.node.get_logger().info(f"Sent Nav2 goal to ({pre_dock_x:.2f}, {pre_dock_y:.2f})")
            return py_trees.common.Status.RUNNING

        if self.goal_done:
            return py_trees.common.Status.SUCCESS if self.goal_success else py_trees.common.Status.FAILURE
            
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID:
            if self.goal_handle is not None and not self.goal_done:
                self.node.get_logger().info("Canceling navigation to cube")
                self.goal_handle.cancel_goal_async()

    def goal_response_callback(self, future):
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.goal_done = True
            self.goal_success = False
            return
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        self.goal_done = True

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.goal_success = True
            self.node.get_logger().info("Nav2 Goal Reached!")
        else:
            self.goal_success = False
            self.node.get_logger().warn(f"Nav2 Goal Failed with status code: {result.status}")


class AlignArmToTargetHight(py_trees.behaviour.Behaviour):
    def __init__(self, name="Align arm to target hight"):
        super().__init__(name=name)

        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key("target_cube_name", access=py_trees.common.Access.READ)

        self.min_arm_position = -0.75
        self.max_arm_position =  0.75
        self.pivot_to_wheel_distance = 0.36

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.pub_arm = self.node.create_publisher(Float64, '/arm_cmd', 10)  # same topic as teleop! Change

        self.tf_buffer = Buffer(node=self.node)
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

    def initialise(self):
        self.target_cube_map = None

    def update(self):
        try:
            if self.target_cube_map is None:
                self.target_cube_map = self.tf_buffer.lookup_transform(
                    'map',
                    self.blackboard.target_cube_name,
                    Time(),
                )

            look_up_time = Time.from_msg(self.target_cube_map.header.stamp)

            if (self.node.get_clock().now() - look_up_time).nanoseconds > 1_000_000_000:
                self.node.get_logger().warning(
                    f'TF data for {self.blackboard.target_cube_name} is too old; waiting for fresh',
                    throttle_duration_sec=1.0,
                )
                self.target_cube_map = None
                return py_trees.common.Status.RUNNING

            arm_map = self.tf_buffer.lookup_transform(
                'map',
                'bagger_arm',
                look_up_time,
            )

        except TransformException as error:
            self.node.get_logger().warning(
                f'Waiting for TF: {error}',
                throttle_duration_sec=1.0,
            )
            return py_trees.common.Status.RUNNING

        if (
            self.target_cube_map.transform.translation.z - arm_map.transform.translation.z
            > math.sin(self.max_arm_position) * self.pivot_to_wheel_distance
            or
            self.target_cube_map.transform.translation.z - arm_map.transform.translation.z
            < math.sin(self.min_arm_position) * self.pivot_to_wheel_distance
        ):
            self.node.get_logger().warning(
                f'Cannot align with {self.blackboard.target_cube_name}: '
                'target height is outside the arm’s allowed height range.'
            )
            return py_trees.common.Status.FAILURE

        target_angle = math.asin(
            (self.target_cube_map.transform.translation.z - arm_map.transform.translation.z) 
            / self.pivot_to_wheel_distance
        )

        msg = Float64()
        msg.data = target_angle
        self.pub_arm.publish(msg)               # add node for handling acceptable limits + ros control joint control
        return py_trees.common.Status.SUCCESS   # publishing angle doesn't mean the goal is reached


class ApproachAndCollectCube(py_trees.behaviour.Behaviour):
    def __init__(self, name="Approach and collect cube"):
        super().__init__(name)

        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key("target_cube_name", py_trees.common.Access.READ)
        self.blackboard.register_key("driven_distance", py_trees.common.Access.WRITE)

        self.angular_kp = 0.5
        self.linear_vel = 0.05
        self.wheel_spin_rate = 3.0
        self.x_start = None
        self.y_start = None

        self.G = 9.8
        self.CONTAINER_MASS = 3.458
        self.CUBE_MASS = 0.1

        self.cubes_mass_in_container = None
        self.cube_count = None
        self.last_mass_change_ns = None

        self.candidate_mass = None
        self.candidate_since_ns = None

        self.mass_change_threshold = 0.01
        self.mass_stability_tolerance = 0.005
        self.mass_stability_duration_ns = 500_000_000

    def setup(self, **kwargs):
        self.node = kwargs['node']

        self.pub_cmd_vel = self.node.create_publisher(TwistStamped, '/cmd_vel_auto', 10)
        self.pub_wheel = self.node.create_publisher(MultiDOFCommand, '/bucket_wheel_cmd_teleop', 10)
        self.sub_container_wheight = self.node.create_subscription(
            WrenchStamped, '/container/wrench', self.container_weight_callback, 10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

    def initialise(self):
        self.target_cube_map = None

        pose = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        self.x_start = pose.transform.translation.x
        self.y_start = pose.transform.translation.y

    def update(self):
        try:
            self.target_cube_map = self.tf_buffer.lookup_transform(
                'map',
                self.blackboard.target_cube_name,
                Time(),
            )

            look_up_time = Time.from_msg(self.target_cube_map.header.stamp)

            bucket_wheel_map = self.tf_buffer.lookup_transform(
                'map',
                'bucket_wheel',
                look_up_time,
            )

            left_wheel_map = self.tf_buffer.lookup_transform(
                'map',
                'left_wheel',
                look_up_time,
            )

            right_wheel_map = self.tf_buffer.lookup_transform(
                'map',
                'right_wheel',
                look_up_time,
            )

        except TransformException as error:
            self.node.get_logger().warning(
                f'Waiting for TF: {error}',
                throttle_duration_sec=1.0,
            )
            return py_trees.common.Status.RUNNING

        diff_drive_position = Vector3()

        diff_drive_position.x = (
            left_wheel_map.transform.translation.x + right_wheel_map.transform.translation.x
        ) / 2
        diff_drive_position.y = (
            left_wheel_map.transform.translation.y + right_wheel_map.transform.translation.y
        ) / 2
        diff_drive_position.z = (
            left_wheel_map.transform.translation.z + right_wheel_map.transform.translation.z
        ) / 2

        diff_drive_to_bucket_wheel_vector = Vector3()
        diff_drive_to_target_cube_vector = Vector3()

        diff_drive_to_bucket_wheel_vector.x = bucket_wheel_map.transform.translation.x - diff_drive_position.x
        diff_drive_to_bucket_wheel_vector.y = bucket_wheel_map.transform.translation.y - diff_drive_position.y
        diff_drive_to_bucket_wheel_vector.z = bucket_wheel_map.transform.translation.z - diff_drive_position.z

        diff_drive_to_target_cube_vector.x = self.target_cube_map.transform.translation.x - diff_drive_position.x
        diff_drive_to_target_cube_vector.y = self.target_cube_map.transform.translation.y - diff_drive_position.y
        diff_drive_to_target_cube_vector.z = self.target_cube_map.transform.translation.z - diff_drive_position.z

        alpha = math.atan2(
            diff_drive_to_bucket_wheel_vector.x * diff_drive_to_target_cube_vector.y -
            diff_drive_to_bucket_wheel_vector.y * diff_drive_to_target_cube_vector.x,
            diff_drive_to_bucket_wheel_vector.x * diff_drive_to_bucket_wheel_vector.y +
            diff_drive_to_target_cube_vector.x * diff_drive_to_target_cube_vector.y
        )

        msg_vel = TwistStamped()
        msg_vel.header.frame_id = "base_link"
        msg_vel.header.stamp = self.node.get_clock().now().to_msg()

        msg_bucket_wheel = MultiDOFCommand()
        msg_bucket_wheel.dof_names = ['bucket_wheel_joint']

        if abs(alpha) > 0.02: 
            msg_vel.twist.angular.z = alpha * self.angular_kp
            msg_bucket_wheel.values = [0.0]
        else:
            msg_vel.twist.linear.x = self.linear_vel
            msg_bucket_wheel.values = [self.wheel_spin_rate]

        if self.cube_count is None or self.cube_count == 0:
            self.pub_cmd_vel.publish(msg_vel)
            self.pub_wheel.publish(msg_bucket_wheel)
            return py_trees.common.Status.RUNNING
        else:
            msg_vel = TwistStamped()
            msg_vel.header.frame_id = "base_link"
            msg_vel.header.stamp = self.node.get_clock().now().to_msg()
            msg_vel.twist.linear.x = 0.0
            msg_vel.twist.angular.z = 0.0
            
            msg_bucket_wheel = MultiDOFCommand()
            msg_bucket_wheel.dof_names = ['bucket_wheel_joint']
            msg_bucket_wheel.values = [0.0]

            self.pub_cmd_vel.publish(msg_vel)           # actions that check that everything has stoped after publishing zero
            self.pub_wheel.publish(msg_bucket_wheel)    # not just hope that everything stops, you should check it

            pose = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.blackboard.driven_distance = math.sqrt(
                (pose.transform.translation.x - self.x_start)**2 + 
                (pose.transform.translation.y - self.y_start)**2
            )

            return py_trees.common.Status.SUCCESS

    def container_weight_callback(self, msg):
        force = msg.wrench.force
        total_force = math.hypot(force.x, force.y, force.z)

        cubes_mass = max(0.0, total_force / self.G - self.CONTAINER_MASS)

        sample_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

        if self.cubes_mass_in_container is not None:
            difference = abs(cubes_mass - self.cubes_mass_in_container)
            if difference <= self.mass_change_threshold:
                self.candidate_mass = None
                self.candidate_since_ns = None
                return

        if (
            self.candidate_mass is None or 
            abs(cubes_mass - self.candidate_mass) > self.mass_stability_tolerance
        ):
            self.candidate_mass = cubes_mass
            self.candidate_since_ns = sample_ns
            return

        if sample_ns - self.candidate_since_ns < self.mass_stability_duration_ns:
            return

        self.cubes_mass_in_container = cubes_mass
        self.cube_count = round(cubes_mass / self.CUBE_MASS)

        self.candidate_mass = None
        self.candidate_since_ns = None


class BackAwayFromHeap(py_trees.behaviour.Behaviour):
    def __init__(self, name="Back away from heap"):
        super().__init__(name=name)

        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key('driven_distance', py_trees.common.Access.READ)

        self.x_start = None
        self.y_start = None

        self.linear_vel = -0.05

    def setup(self, **kwargs):
        self.node = kwargs['node']

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        self.pub_cmd_vel = self.node.create_publisher(TwistStamped, '/cmd_vel_auto', 10)

    def initialise(self):
        pose = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())

        self.x_start = pose.transform.translation.x
        self.y_start = pose.transform.translation.y

    def update(self):
        pose = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())

        if (
            (pose.transform.translation.x - self.x_start)**2 +
            (pose.transform.translation.y - self.y_start)**2 < self.blackboard.driven_distance**2
        ):
            msg_vel = TwistStamped()
            msg_vel.header.frame_id = "base_link"
            msg_vel.header.stamp = self.node.get_clock().now().to_msg()
            msg_vel.twist.linear.x = self.linear_vel
            self.pub_cmd_vel.publish(msg_vel)

            return py_trees.common.Status.RUNNING
        else:
            msg_vel = TwistStamped()
            msg_vel.header.frame_id = "base_link"
            msg_vel.header.stamp = self.node.get_clock().now().to_msg()
            msg_vel.twist.linear.x = 0.0
            self.pub_cmd_vel.publish(msg_vel)       # make sure uve stopped before reporting success

            return py_trees.common.Status.SUCCESS


class NavigateToDumpingArea(py_trees.behaviour.Behaviour):
    def __init__(self, name="Navigate to cube"):
        super().__init__(name)

        self.goal_sent = False
        self.goal_done = False
        self.goal_success = False

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.nav_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')

    def initialise(self):
        self.goal_sent = False
        self.goal_done = False
        self.goal_success = False
        self.goal_handle = None

    def update(self):
        if not self.goal_sent:
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()

            goal_msg.pose.pose.position.x = 13.9
            goal_msg.pose.pose.position.y = 0.0
            goal_msg.pose.pose.position.z = 0.0

            goal_msg.pose.pose.orientation.x = 0.0
            goal_msg.pose.pose.orientation.y = 0.0
            goal_msg.pose.pose.orientation.z = 1.0
            goal_msg.pose.pose.orientation.w = 0.0

            self.logger.info('Before waiting')
            if not self.nav_client.server_is_ready():
                self.node.get_logger().warn("Waiting for Nav2 action server...")
                return py_trees.common.Status.RUNNING

            send_goal_future = self.nav_client.send_goal_async(goal_msg)
            send_goal_future.add_done_callback(self.goal_response_callback)
            
            self.goal_sent = True
            self.node.get_logger().info(
                f"Sent Nav2 goal to ({goal_msg.pose.pose.position.x:.2f}, "
                f"{goal_msg.pose.pose.position.y:.2f})"
            )
            return py_trees.common.Status.RUNNING

        if self.goal_done:
            return py_trees.common.Status.SUCCESS if self.goal_success else py_trees.common.Status.FAILURE
            
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID:
            if self.goal_handle is not None and not self.goal_done:
                self.node.get_logger().info("Canceling navigation to cube")
                self.goal_handle.cancel_goal_async()

    def goal_response_callback(self, future):
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.goal_done = True
            self.goal_success = False
            return
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        self.goal_done = True

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.goal_success = True
            self.node.get_logger().info("Nav2 Goal Reached!")
        else:
            self.goal_success = False
            self.node.get_logger().warn(f"Nav2 Goal Failed with status code: {result.status}")


def create_tree():
    root = py_trees.composites.Sequence(name="Cube Collection Task", memory=True)

    wait_for_cube_pose_behavior = WaitForCubePose()
    navigate_to_cube_behavior = NavigateToCube()
    aling_arm_to_target_hight_behavior = AlignArmToTargetHight()
    approch_and_collect_cube_behavior = ApproachAndCollectCube()
    back_away_from_heap_behavior = BackAwayFromHeap()
    navigate_to_dumping_area_behavior = NavigateToDumpingArea()
    
    root.add_children([
        wait_for_cube_pose_behavior,
        navigate_to_cube_behavior, 
        aling_arm_to_target_hight_behavior,
        approch_and_collect_cube_behavior,
        back_away_from_heap_behavior,
        navigate_to_dumping_area_behavior,
    ])
    return root

def main():
    rclpy.init()

    root = create_tree()
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
    )
    
    tree.setup(timeout=15.0)

    blackboard = py_trees.blackboard.Client(name="Main")
    blackboard.register_key("target_cube_name", access=py_trees.common.Access.WRITE)
    
    blackboard.target_cube_name = 'cube_2'

    try:
        tree.tick_tock(period_ms=100.0)

        while rclpy.ok():
            rclpy.spin_once(tree.node, timeout_sec=0.1)

            if tree.root.status in (py_trees.common.Status.SUCCESS, py_trees.common.Status.FAILURE):
                tree.node.get_logger().info(
                    py_trees.display.unicode_tree(root=tree.root, show_status=True)
                )
                break

    except KeyboardInterrupt:
        pass

    finally:
        tree.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
