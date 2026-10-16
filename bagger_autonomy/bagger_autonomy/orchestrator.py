import math
import time
from typing import Any

import py_trees
import py_trees_ros
import rclpy
import tf2_geometry_msgs
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
from nav2_msgs.action import NavigateToPose
from py_trees.common import Status
from rclpy.action import ActionClient
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Bool, Float64
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.time import Time


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
            
            pre_dock_x = cx - 0.90 * math.cos(angle_to_cube)
            pre_dock_y = cy - 0.90 * math.sin(angle_to_cube)

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
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


# =====================================================================
# BEHAVIOR 2: Visual Servo until Contact Sensor Triggers
# =====================================================================
class VisualServoToContact(py_trees.behaviour.Behaviour):
    def __init__(self, name="Visual Servo"):
        super().__init__(name)

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.cmd_vel_pub = self.node.create_publisher(TwistStamped, 'cmd_vel_auto', 10)
        self.contact_sub = self.node.create_subscription(Contacts, '/bucket_wheel/contacts', self.contact_callback, 10)
        self.tag_sub = self.node.create_subscription(PoseStamped, 'apriltag_pose', self.tag_callback, 10)
        
        self.contact_made = False
        self.tag_y_offset = 0.0
        self.logger.info('Setup successful!')

    def contact_callback(self, msg: Bool):
        self.contact_made = msg.data

    def tag_callback(self, msg: PoseStamped):
        # Assuming Y is the horizontal offset in your camera optical frame
        self.tag_y_offset = msg.pose.position.y

    def initialise(self):
        self.contact_made = False
        self.node.get_logger().info("Starting Visual Servoing phase...")

    def update(self):
        # Success condition: We hit the cube
        if self.contact_made:
            self.node.get_logger().info("Contact Detected! Docking successful.")
            self.cmd_vel_pub.publish(TwistStamped()) # Stop
            return py_trees.common.Status.SUCCESS

        # Active control: Drive slow, steer towards tag
        twist = TwistStamped()
        twist.linear.x = 0.05  # 5 cm/s
        Kp = 0.5               # Proportional gain for steering
        twist.angular.z = Kp * self.tag_y_offset
        
        self.cmd_vel_pub.publish(twist)
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        # Safety catch: Always stop wheels when this behavior exits or is canceled
        self.cmd_vel_pub.publish(TwistStamped())


def create_tree():
    root = py_trees.composites.Sequence(name="Cube Collection Task", memory=True)

    wait_for_cube_pose_behavior = WaitForCubePose()
    navigate_to_cube_behavior = NavigateToCube()
    aling_arm_to_target_hight_behavior = AlignArmToTargetHight()
    servo_behavior = VisualServoToContact()
    
    root.add_children([wait_for_cube_pose_behavior, navigate_to_cube_behavior, aling_arm_to_target_hight_behavior])
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
        while rclpy.ok():
            tree.tick()

            if tree.root.status in (py_trees.common.Status.SUCCESS, py_trees.common.Status.FAILURE):
                tree.node.get_logger().info(py_trees.display.unicode_tree(root=tree.root, show_status=True))
                break

            rclpy.spin_once(tree.node, timeout_sec=0.1)

    except KeyboardInterrupt:
        pass

    finally:
        tree.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
