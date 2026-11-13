import math
import time
import signal
from pathlib import Path
from textwrap import indent
from abc import abstractmethod

import py_trees
from py_trees.common import Status
import py_trees_ros
import rclpy
import tf2_geometry_msgs
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TwistStamped, Vector3, WrenchStamped, TransformStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.time import Time
from control_msgs.msg import MultiDOFCommand
from rclpy.signals import SignalHandlerOptions
from rclpy.serialization import serialize_message
from rosidl_runtime_py.convert import message_to_yaml

from bagger_interfaces.action import ExecuteDumping, TargetRecovery
from bagger_interfaces.msg import CubePoseWithMaskArray


class ActionClientBehaviour(py_trees.behaviour.Behaviour):
    def initialise(self):
        self.send_goal_future = None
        self.goal_handle = None
        self.result_response = None
        self.cancel_requested = False

    def update(self):
        if self.goal_handle is not None and not self.goal_handle.accepted:
            return py_trees.common.Status.FAILURE

        if self.result_response is not None:
            if self.result_response.status == GoalStatus.STATUS_SUCCEEDED:
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID:
            if self.goal_handle is not None and self.result_response is None and self.goal_handle.accepted:
                self.goal_handle.cancel_goal_async()

            self.cancel_requested = True

    def goal_response_callback(self, future):
        self.goal_handle = future.result()

        if self.goal_handle.accepted:
            if self.cancel_requested:
                self.goal_handle.cancel_goal_async()

            result_future = self.goal_handle.get_result_async()
            result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.result_response = future.result()


class SelectNextCube(py_trees.behaviour.Behaviour):
    def __init__(self, name="Select next cube"):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key("target_cube_position", access=py_trees.common.Access.WRITE)

        self.deadline = None
        self.all_collected = False

        self.num_cubes = 100
        self.cube_poses = {f'cube_{i}': None for i in range(self.num_cubes)}

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.sub_cube_poses = self.node.create_subscription(TFMessage, '/ground_truth/cubes', self.cube_poses_callback, 10)


    def initialise(self):
        self.blackboard.target_cube_position = None
        self.cube_poses = {f'cube_{i}': None for i in range(self.num_cubes)}
        self.deadline = self.node.get_clock().now() + Duration(seconds=60.0)

    def update(self):
        if self.blackboard.target_cube_position is not None:
            return py_trees.common.Status.SUCCESS
        elif self.node.get_clock().now() >= self.deadline:
            self.node.get_logger().error('Time out waiting for target cube pose')
            return py_trees.common.Status.FAILURE
        else:
            has_undetected_cubes = False

            for cube_id, position in self.cube_poses.items():
                if position is None:
                    has_undetected_cubes = True
                else:
                    if not (15.0 <= position.x <= 19.0) or not (-5.0 <= position.y <= 5.0):
                        self.all_collected = False
                        self.blackboard.target_cube_position = position
                        return py_trees.common.Status.SUCCESS
                    
            if has_undetected_cubes:
                return py_trees.common.Status.RUNNING
            
            self.all_collected = True
            return py_trees.common.Status.FAILURE

    def cube_poses_callback(self, msg):
        for transform in msg.transforms:
            if transform.child_frame_id in self.cube_poses:
                self.cube_poses[transform.child_frame_id] = transform.transform.translation
                return


class NavigateToCube(ActionClientBehaviour):
    def __init__(self, name="Navigate to cube"):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key("target_cube_position", access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.nav_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

    def update(self):
        if self.send_goal_future is None:
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
            
            pre_dock_x = cx - 1.10 * math.cos(angle_to_cube)
            pre_dock_y = cy - 1.10 * math.sin(angle_to_cube)

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

            self.send_goal_future = self.nav_client.send_goal_async(goal_msg)
            self.send_goal_future.add_done_callback(self.goal_response_callback)
            
            self.node.get_logger().info(f"Sent Nav2 goal to ({pre_dock_x:.2f}, {pre_dock_y:.2f})")
            return py_trees.common.Status.RUNNING

        return super().update()


class AlignArmToTargetHeight(py_trees.behaviour.Behaviour):
    def __init__(self, name="Align arm to target height"):
        super().__init__(name=name)

        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key("detected_target_position", access=py_trees.common.Access.WRITE)

        self.min_arm_position = -0.75
        self.max_arm_position =  0.75
        self.pivot_to_wheel_distance = 0.36
        self.target_detection_timeout = Duration(seconds=10)

    def setup(self, **kwargs):
        self.node = kwargs['node']

        self.pub_arm = self.node.create_publisher(Float64, '/arm_cmd', 10)  # same topic as teleop! Change

        self.sub_detected_targets = self.node.create_subscription(
            CubePoseWithMaskArray,
            '/detected_targets',
            self.detected_target_position_callback,
            10,
        )

        self.detected_targets = None

        self.tf_buffer = Buffer(node=self.node)
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

    def initialise(self):
        self.detected_targets = None
        self.blackboard.detected_target_position = None
        self.target_detection_start_time = self.node.get_clock().now()

    def update(self):
        try:
            if self.blackboard.detected_target_position is None:
                elapsed = self.node.get_clock().now() - self.target_detection_start_time
                if elapsed >= self.target_detection_timeout:
                    self.node.get_logger().error("Timeout reached. No target detected")
                    return py_trees.common.Status.FAILURE

                if self.detected_targets is None or not self.detected_targets.cubes:
                    return py_trees.common.Status.RUNNING

                detection = self.detected_targets.cubes[0]

                self.tf_buffer.set_transform(detection.transform, "target_position_estimation")     # use later self.tf_buffer.transform

                self.blackboard.detected_target_position = self.tf_buffer.lookup_transform(
                    'odom',
                    detection.transform.child_frame_id,
                    Time.from_msg(detection.transform.header.stamp),
                )
                
                path = Path.home() / "my_projects/lunar_bagger/src/detected_targets.cdr"

                if self.detected_targets is not None:
                    path.write_bytes(serialize_message(self.detected_targets))

            self.node.get_logger().info(f'self.blackboard.detected_target_position.transform.translation: {self.blackboard.detected_target_position.transform.translation}\n')

            arm_map = self.tf_buffer.lookup_transform(
                'odom',
                'bagger_arm',
                Time(),
            )

        except TransformException as error:
            self.node.get_logger().warning(
                f'Waiting for TF: {error}',
                throttle_duration_sec=1.0,
            )
            return py_trees.common.Status.RUNNING

        if (
            self.blackboard.detected_target_position.transform.translation.z - arm_map.transform.translation.z
            > math.sin(self.max_arm_position) * self.pivot_to_wheel_distance
            or
            self.blackboard.detected_target_position.transform.translation.z - arm_map.transform.translation.z
            < math.sin(self.min_arm_position) * self.pivot_to_wheel_distance
        ):
            self.node.get_logger().warning(
                f'Cannot align with detected_target_0 at z = {self.blackboard.detected_target_position.transform.translation.z}: '
                'target height is outside the arm’s allowed height range.'
            )
            return py_trees.common.Status.FAILURE

        target_angle = math.asin(
            (self.blackboard.detected_target_position.transform.translation.z - arm_map.transform.translation.z) 
            / self.pivot_to_wheel_distance
        )

        msg = Float64()
        msg.data = target_angle
        self.pub_arm.publish(msg)               # add node for handling acceptable limits + ros control joint control
        return py_trees.common.Status.SUCCESS   # publishing angle doesn't mean the goal is reached

    def detected_target_position_callback(self, msg):
        self.detected_targets = msg


class ApproachAndCollectCube(py_trees.behaviour.Behaviour):
    def __init__(self, name="Approach and collect cube"):
        super().__init__(name)

        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key("detected_target_position", access=py_trees.common.Access.READ)
        self.blackboard.register_key("driven_distance", py_trees.common.Access.WRITE)

        self.angular_kp = 0.2
        self.linear_vel = 0.05
        self.wheel_spin_rate = 3.0
        self.x_start = None
        self.y_start = None

        self.G = 9.8
        self.CONTAINER_MASS = 3.458
        self.CUBE_MASS = 0.1

        self.cubes_mass_in_container = None
        self.cube_count = None

        self.candidate_mass = None
        self.candidate_since_ns = None

        self.mass_change_threshold = 0.01
        self.mass_stability_tolerance = 0.005
        self.mass_stability_duration_ns = 500_000_000

    def setup(self, **kwargs):
        self.node = kwargs['node']

        self.pub_cmd_vel = self.node.create_publisher(TwistStamped, '/cmd_vel_auto', 10)
        self.pub_wheel = self.node.create_publisher(MultiDOFCommand, '/bucket_wheel_cmd_autonomy', 10)
        self.sub_container_weight = self.node.create_subscription(
            WrenchStamped, '/container/wrench', self.container_weight_callback, 10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        self.start_gt = None
        self.end_gt = None
        self.node.create_subscription(
            TFMessage, '/ground_truth/pose', self.gt_callback, 10,
        )

    def initialise(self):
        self.bucket_wheel_map = None

        pose = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
        self.x_start = pose.transform.translation.x
        self.y_start = pose.transform.translation.y
        self.start_tf = pose.transform
        self.start_gt = None

        self.bucket_wheel_to_target_distance_increases_count = 0
        self.min_bucket_wheel_to_target_distance = None

    def update(self):
        self.node.get_logger().info(f'self.blackboard.detected_target_position.transform.translation: {self.blackboard.detected_target_position.transform.translation}\n')
        try:
            if self.bucket_wheel_map is None:
                self.bucket_wheel_map = self.tf_buffer.lookup_transform(
                    'odom',
                    'bucket_wheel',
                    Time(),
                )

            look_up_time = Time.from_msg(self.bucket_wheel_map.header.stamp)

            if (self.node.get_clock().now() - look_up_time).nanoseconds > 1_000_000_000:
                self.node.get_logger().warning(
                    f'TF data for bucket_wheel_map is too old; waiting for fresh',
                    throttle_duration_sec=1.0,
                )
                self.bucket_wheel_map = None
                return py_trees.common.Status.RUNNING

            left_wheel_map = self.tf_buffer.lookup_transform(
                'odom',
                'left_wheel',
                look_up_time,
            )

            right_wheel_map = self.tf_buffer.lookup_transform(
                'odom',
                'right_wheel',
                look_up_time,
            )

        except TransformException as error:
            self.node.get_logger().warning(
                f'Waiting for TF: {error}',
                throttle_duration_sec=1.0,
            )
            return py_trees.common.Status.RUNNING

        if self.bucket_wheel_to_target_distance_increases_count > 25:
            return py_trees.common.Status.FAILURE

        diff_drive_position = Vector3()

        diff_drive_position.x = (
            left_wheel_map.transform.translation.x + right_wheel_map.transform.translation.x
        ) / 2
        diff_drive_position.y = (
            left_wheel_map.transform.translation.y + right_wheel_map.transform.translation.y
        ) / 2

        if self.min_bucket_wheel_to_target_distance is None:
            self.min_bucket_wheel_to_target_distance = math.sqrt(
                (self.blackboard.detected_target_position.transform.translation.x - self.bucket_wheel_map.transform.translation.x)**2 +
                (self.blackboard.detected_target_position.transform.translation.y - self.bucket_wheel_map.transform.translation.y)**2
            )

        else:
            bucket_wheel_to_target_distance = math.sqrt(
                (self.blackboard.detected_target_position.transform.translation.x - self.bucket_wheel_map.transform.translation.x)**2 +
                (self.blackboard.detected_target_position.transform.translation.y - self.bucket_wheel_map.transform.translation.y)**2
            )

            if bucket_wheel_to_target_distance - 0.05 > self.min_bucket_wheel_to_target_distance:
                self.bucket_wheel_to_target_distance_increases_count += 1


            self.node.get_logger().info(f'left x: {left_wheel_map.transform.translation.x}')
            self.node.get_logger().info(f'right x: {right_wheel_map.transform.translation.x}')
            self.node.get_logger().info(f'left y: {left_wheel_map.transform.translation.y}')
            self.node.get_logger().info(f'right y: {right_wheel_map.transform.translation.y}')
            self.node.get_logger().info(f'count: {self.bucket_wheel_to_target_distance_increases_count}')
            self.node.get_logger().info(f'min: {self.min_bucket_wheel_to_target_distance}')
            self.node.get_logger().info(f'current: {bucket_wheel_to_target_distance}')

            self.min_bucket_wheel_to_target_distance = min(bucket_wheel_to_target_distance, self.min_bucket_wheel_to_target_distance)

        diff_drive_to_bucket_wheel_vector = Vector3()
        diff_drive_to_detected_target_vector = Vector3()

        diff_drive_to_bucket_wheel_vector.x = self.bucket_wheel_map.transform.translation.x - diff_drive_position.x
        diff_drive_to_bucket_wheel_vector.y = self.bucket_wheel_map.transform.translation.y - diff_drive_position.y

        diff_drive_to_detected_target_vector.x = self.blackboard.detected_target_position.transform.translation.x - diff_drive_position.x
        diff_drive_to_detected_target_vector.y = self.blackboard.detected_target_position.transform.translation.y - diff_drive_position.y

        alpha = math.atan2(
            diff_drive_to_bucket_wheel_vector.x * diff_drive_to_detected_target_vector.y -
            diff_drive_to_bucket_wheel_vector.y * diff_drive_to_detected_target_vector.x,
            diff_drive_to_bucket_wheel_vector.x * diff_drive_to_detected_target_vector.x +
            diff_drive_to_bucket_wheel_vector.y * diff_drive_to_detected_target_vector.y
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

            self.pub_cmd_vel.publish(msg_vel)           # actions that check that everything has stopped after publishing zero
            self.pub_wheel.publish(msg_bucket_wheel)    # not just hope that everything stops, you should check it

            pose = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())          # use diff drive center for this distnace
            self.blackboard.driven_distance = math.sqrt(
                (pose.transform.translation.x - self.x_start)**2 + 
                (pose.transform.translation.y - self.y_start)**2
            )

            pose = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            self.end_tf = pose.transform

            values = {
                "start_tf": self.start_tf,
                "end_tf": self.end_tf,
                "start_gt": self.start_gt,
                "end_gt": self.end_gt,
                "diff_drive_position": self.blackboard.driven_distance,
            }

            path = Path.home() / "my_projects/lunar_bagger/src/collection_debug.yaml"

            with path.open("w") as file:
                for name, value in values.items():
                    if value is None:
                        text = "null\n"
                    elif isinstance(value, (int, float)):
                        text = f"{value}\n"
                    else:
                        text = message_to_yaml(value)
                    file.write(f"{name}:\n{indent(text, '  ')}\n")

            return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID:
            msg_vel = TwistStamped()
            self.pub_cmd_vel.publish(msg_vel)

            msg_bucket_wheel = MultiDOFCommand()
            msg_bucket_wheel.dof_names = ['bucket_wheel_joint']
            msg_bucket_wheel.values = [0.0]
            self.pub_wheel.publish(msg_bucket_wheel)

            pose = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            self.end_tf = pose.transform

            values = {
                "start_tf": self.start_tf,
                "end_tf": self.end_tf,
                "start_gt": self.start_gt,
                "end_gt": self.end_gt,
                "diff_drive_position": self.blackboard.driven_distance,
            }

            path = Path.home() / "my_projects/lunar_bagger/src/collection_debug.yaml"

            with path.open("w") as file:
                for name, value in values.items():
                    if value is None:
                        text = "null\n"
                    elif isinstance(value, (int, float)):
                        text = f"{value}\n"
                    else:
                        text = message_to_yaml(value)
                    file.write(f"{name}:\n{indent(text, '  ')}\n")


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

    def gt_callback(self, msg):
        for transform in msg.transforms:
            if transform.header.frame_id == 'warehouse_world' and transform.child_frame_id == 'bagger':
                if self.start_gt is None:
                    self.start_gt = transform
                self.end_gt = transform


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
        self.pub_arm = self.node.create_publisher(Float64, '/arm_cmd', 10)  # same topic as teleop! Change

    def initialise(self):
        pose = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())

        self.x_start = pose.transform.translation.x
        self.y_start = pose.transform.translation.y

        msg = Float64()
        msg.data = -0.2
        self.pub_arm.publish(msg)               # add node for handling acceptable limits + ros control joint control
        return py_trees.common.Status.SUCCESS   # publishing angle doesn't mean the goal is reached

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
            self.pub_cmd_vel.publish(msg_vel)       # make sure you've stopped before reporting success

            return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID:
            msg_vel = TwistStamped()
            self.pub_cmd_vel.publish(msg_vel)


class NavigateToDumpingArea(ActionClientBehaviour):
    def __init__(self, name="Navigate to dumping area"):
        super().__init__(name)

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.nav_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')

    def update(self):
        if self.send_goal_future is None:
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()

            goal_msg.pose.pose.position.x = 14.1
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

            self.send_goal_future = self.nav_client.send_goal_async(goal_msg)
            self.send_goal_future.add_done_callback(self.goal_response_callback)
            
            self.node.get_logger().info(
                f"Sent Nav2 goal to ({goal_msg.pose.pose.position.x:.2f}, "
                f"{goal_msg.pose.pose.position.y:.2f})"
            )
            return py_trees.common.Status.RUNNING

        return super().update()


class DumpContainer(ActionClientBehaviour):
    def __init__(self, name='Dump container'):
        super().__init__(name=name)

    def setup(self, **kwargs):
        self.node = kwargs["node"]

        self.action_client = ActionClient(
            self.node,
            ExecuteDumping,
            "/execute_dumping"
        )

    def update(self):
        if self.send_goal_future is None:
            goal_msg = ExecuteDumping.Goal()

            self.send_goal_future = self.action_client.send_goal_async(goal_msg)
            self.send_goal_future.add_done_callback(self.goal_response_callback)

            return py_trees.common.Status.RUNNING

        return super().update()


class FailureRecovery(ActionClientBehaviour):
    def __init__(self, name="Failure recovery"):
        super().__init__(name)

        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key("detected_target_position", access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        self.node = kwargs["node"]

        self.action_client = ActionClient(
            self.node,
            TargetRecovery,
            "target_recovery",
        )

    def update(self):
        if self.send_goal_future is None:
            goal_msg = TargetRecovery.Goal()

            self.send_goal_future = self.action_client.send_goal_async(goal_msg)
            self.send_goal_future.add_done_callback(self.goal_response_callback)

            return py_trees.common.Status.RUNNING

        return super().update()


class CollectAllCubes(py_trees.decorators.Decorator):
    def __init__(self, child, selection, name="Collect all cubes"):
        super().__init__(name=name, child=child)
        self.selection = selection

    def update(self):
        if self.selection.all_collected:
            return py_trees.common.Status.SUCCESS

        if self.decorated.status == py_trees.common.Status.SUCCESS:
            return py_trees.common.Status.RUNNING

        return self.decorated.status


def create_tree():
    selection = SelectNextCube()
    
    attempt = py_trees.composites.Sequence(
        name="Attempt",
        memory=True,
        children=[AlignArmToTargetHeight(), ApproachAndCollectCube(),],
    )

    recovery = py_trees.decorators.SuccessIsFailure(
        name="Recovery",
        child=FailureRecovery(),
    )

    attempt_or_recovery = py_trees.decorators.FailureIsRunning(
        name="Attemp or recovery",
        child= py_trees.composites.Selector(
            name="Selector",
            memory=True,
            children=[
                attempt,
                recovery,
            ],
        ),
    )

    cycle = py_trees.composites.Sequence(
        name="Cube Collection Task",
        memory=True,
        children=[
            selection,
            NavigateToCube(),
            attempt_or_recovery,
            BackAwayFromHeap(),
            NavigateToDumpingArea(),
            DumpContainer(),
        ],
    )

    return CollectAllCubes(child=cycle, selection=selection)

def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    stop_requested = False

    def request_stop(signum, frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

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

        while rclpy.ok() and not stop_requested:
            rclpy.spin_once(tree.node, timeout_sec=0.1)

            if tree.root.status in (py_trees.common.Status.SUCCESS, py_trees.common.Status.FAILURE):
                tree.node.get_logger().info(
                    py_trees.display.unicode_tree(root=tree.root, show_status=True)
                )
                break

    except KeyboardInterrupt:
        pass

    finally:
        if tree.timer is not None:
            tree.timer.cancel()

        actions = [
            behaviour
            for behaviour in root.iterate()
            if getattr(behaviour, 'send_goal_future', None) is not None
        ]

        try:
            root.stop(py_trees.common.Status.INVALID)

            deadline = time.monotonic() + 10.0

            while rclpy.ok():
                pending = [
                    behaviour
                    for behaviour in actions
                    if behaviour.goal_handle is None
                    or (
                        behaviour.goal_handle.accepted
                        and behaviour.result_response is None
                    )
                ]

                if not pending:
                    break

                if time.monotonic() >= deadline:
                    tree.node.get_logger().warning(
                        'Shutdown timed out; action completion was not confirmed.'
                    )
                    break

                rclpy.spin_once(tree.node, timeout_sec=0.1)

        finally:
            tree.shutdown()
            rclpy.try_shutdown()

if __name__ == '__main__':
    main()
