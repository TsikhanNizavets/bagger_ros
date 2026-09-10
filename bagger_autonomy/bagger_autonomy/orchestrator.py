import rclpy
import math
import py_trees
import py_trees_ros

from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion
from std_msgs.msg import Bool
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from ros_gz_interfaces.msg import Contacts
from tf2_ros import Buffer, TransformListener

# =====================================================================
# BEHAVIOR 1: Navigate to 30cm Pre-Dock Pose
# =====================================================================
class ApproachCubeNav2(py_trees.behaviour.Behaviour):
    def __init__(self, name="Approach Nav2"):
        super().__init__(name)
        # Use Blackboard to share data (like target coordinates) between behaviors
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key("cube_target", access=py_trees.common.Access.READ)
        
        self.goal_sent = False
        self.goal_done = False
        self.goal_success = False

    def setup(self, **kwargs):
        # py_trees_ros passes the ROS 2 node in kwargs during setup
        self.node = kwargs['node']
        self.nav_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        
        # TF Setup to get robot's current pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

    def initialise(self):
        # Reset state every time this behavior starts
        self.goal_sent = False
        self.goal_done = False
        self.goal_success = False

    def update(self):
        # 1. Calculate and send goal if we haven't yet
        if not self.goal_sent:
            target_pose = self.blackboard.cube_target
            if not target_pose:
                self.node.get_logger().warn("No target in blackboard yet!")
                return py_trees.common.Status.FAILURE

            try:
                # Find where the robot currently is
                trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                rx = trans.transform.translation.x
                ry = trans.transform.translation.y
            except Exception as e:
                self.node.get_logger().warn(f"Waiting for TF: {e}")
                return py_trees.common.Status.RUNNING # Keep trying

            # Calculate 30cm offset
            cx = target_pose.pose.position.x
            cy = target_pose.pose.position.y
            angle_to_cube = math.atan2(cy - ry, cx - rx)
            
            pre_dock_x = cx - 0.30 * math.cos(angle_to_cube)
            pre_dock_y = cy - 0.30 * math.sin(angle_to_cube)

            # Build Nav2 Goal
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.pose.position.x = pre_dock_x
            goal_msg.pose.pose.position.y = pre_dock_y
            
            # Manual quaternion calculation for Z-rotation (Yaw)
            goal_msg.pose.pose.orientation.z = math.sin(angle_to_cube / 2.0)
            goal_msg.pose.pose.orientation.w = math.cos(angle_to_cube / 2.0)

            self.logger.info('Before waiting')
            # Send Goal Async
            if not self.nav_client.server_is_ready():
                self.node.get_logger().warn("Waiting for Nav2 action server...")
                return py_trees.common.Status.RUNNING

            send_goal_future = self.nav_client.send_goal_async(goal_msg)
            send_goal_future.add_done_callback(self.goal_response_callback)
            
            self.goal_sent = True
            self.node.get_logger().info(f"Sent Nav2 goal to ({pre_dock_x:.2f}, {pre_dock_y:.2f})")
            return py_trees.common.Status.RUNNING

        # 2. Check if action is finished
        if self.goal_done:
            return py_trees.common.Status.SUCCESS if self.goal_success else py_trees.common.Status.FAILURE
            
        return py_trees.common.Status.RUNNING

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.goal_done = True
            self.goal_success = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # Nav2 finished
        result = future.result()
        self.goal_done = True

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.goal_success = True
            self.node.get_logger().info("Nav2 Goal Reached!")
        else:
            self.goal_success = False
            self.node.get_logger().warn(f"Nav2 Goal Failed with status code: {result.status}")

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


# =====================================================================
# MAIN TREE EXECUTION
# =====================================================================
def create_tree():
    # A Sequence ticks its children in order. 
    # If child 1 succeeds, it ticks child 2.
    root = py_trees.composites.Sequence(name="Cube Collection Task", memory=True)
    
    nav_behavior = ApproachCubeNav2()
    servo_behavior = VisualServoToContact()
    
    root.add_children([nav_behavior, servo_behavior])
    # root.add_child(nav_behavior)
    return root

def main():
    rclpy.init()

    # Create the Behavior Tree and ROS wrapper
    root = create_tree()
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
    )
    
    # 1. Setup the tree (this calls setup() on all our behaviors and passes the ROS node)
    tree.setup(timeout=15.0)

    # 2. Simulate receiving a target (In reality, another node would set this)
    # We write it to the Blackboard so the Nav2 behavior can read it.
    blackboard = py_trees.blackboard.Client(name="Main")
    blackboard.register_key("cube_target", access=py_trees.common.Access.WRITE)
    
    target = PoseStamped()
    target.pose.position.x = 4.0
    target.pose.position.y = 4.0
    blackboard.cube_target = target

    # 3. Tick the tree continually at 10Hz
    try:
        while rclpy.ok():
            tree.tick()

            all_done = all(
                child.status in (py_trees.common.Status.SUCCESS, py_trees.common.Status.FAILURE)
                for child in tree.root.children
            )

            if all_done:
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
