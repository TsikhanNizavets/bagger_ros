"""
Launch only from dump_navigator.launch.py

No launch from bringup

TODO: This should be orchestrator node. Controls mining node, dumping node and nav2. Starts from teleop
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')     # use /navigate_to_pose?

    def send_goal(self, x, y, theta):
        self.get_logger().info('Waiting for Nav2 action server...')
        self.action_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0               # not zero but just doesn't change? i don't know if the current position is zero

        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(theta / 2.0)        # quartenios - are they set correctly?
        goal_msg.pose.pose.orientation.w = math.cos(theta / 2.0)

        self.get_logger().info(f'Sending truck to X: {x}, Y: {y}')

        self.send_goal_future = self.action_client.send_goal_async(goal_msg)            # send_goal() vs send_goal_async?
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected by server')
            rclpy.shutdown()
            return
        self.get_logger().info('Goal accepted, navigating...')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        self.get_logger().info(f'Reached dumping area! Status code: {status}')

        rclpy.shutdown()

def main(args=None):
    try:
        rclpy.init(args=args)
        node = MissionController()
        node.send_goal(x=5.0, y=-2.5, theta=1.57)           # Only same values. make them custom.
        rclpy.spin(node)                        # Why first send a goal and only then spin?
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
