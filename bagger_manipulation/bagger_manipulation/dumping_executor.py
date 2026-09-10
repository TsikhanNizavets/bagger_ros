"""
Action. Drives backward, when distance jumps, stops, unloads, and drives back in the safety zone

TODO: Info messages in terminal don't really help and are not informative. Either redesing or remove
TODO: Reject new goal if already executing one. Should i implement same for container_unloader? or only on this level?
"""

import math
import rclpy
from rclpy.task import Future
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.node import Node
from bagger_interfaces.action import ExecuteDumping, UnloadContainer
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class DumpingExecutor(Node):
    def __init__(self):
        super().__init__('dumping_executor')

        self.drop_off_distance = 0
        self.linear_speed = 0.1
        self.current_x = 0.0
        self.current_y = 0.0
        self.is_executing = False

        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.pub_cmd_vel = self.create_publisher(TwistStamped, '/cmd_vel_auto', 10)
        self.sub_left_drop_off = self.create_subscription(
            LaserScan, '/left_drop_off/scan', self.drop_off_callback, 10
        )
        self.sub_right_drop_off = self.create_subscription(
            LaserScan, '/right_drop_off/scan', self.drop_off_callback, 10
        )

        self.action_server = ActionServer(
            self,
            ExecuteDumping,
            'execute_dumping',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.action_client = ActionClient(
            self,
            UnloadContainer,
            '/unload_container',
            )

        self.get_logger().info('DumpingExecutor ready on /execute_dumping')

    def goal_callback(self, goal_request):
        if self.is_executing:
            self.get_logger().warn('Rejecting goal: Already executing.')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request!')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.is_executing = True
        self.get_logger().info('Executing dumping action...')

        feedback_msg = ExecuteDumping.Feedback()
        result = ExecuteDumping.Result()

        feedback_msg.current_state = 'DOCKING'
        goal_handle.publish_feedback(feedback_msg)

        cmd_msg = TwistStamped()
        cmd_msg.header.frame_id = "base_link"

        start_x = self.current_x
        start_y = self.current_y
        driven_distance = 0.0

        while self.drop_off_distance < 0.7:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.is_executing = False
                return result

            cmd_msg.header.stamp = self.get_clock().now().to_msg()
            cmd_msg.twist.linear.x = -self.linear_speed
            self.pub_cmd_vel.publish(cmd_msg)

            driven_distance = math.hypot(self.current_x - start_x, self.current_y - start_y)
            self.get_logger().info(f'{driven_distance}')

            await self.async_sleep(0.1)

        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.twist.linear.x = 0.0
        self.pub_cmd_vel.publish(cmd_msg)

        feedback_msg.current_state = 'UNLOADING'
        goal_handle.publish_feedback(feedback_msg)

        await self.trigger_unload()

        return_start_x = self.current_x
        return_start_y = self.current_y
        return_distance = 0.0

        while return_distance < driven_distance:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.is_executing = False
                return result
            
            cmd_msg.header.stamp = self.get_clock().now().to_msg()
            cmd_msg.twist.linear.x = self.linear_speed
            self.pub_cmd_vel.publish(cmd_msg)

            return_distance = math.hypot(return_start_x - self.current_x, return_start_y - self.current_y)

            await self.async_sleep(0.1)

        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.twist.linear.x = 0.0
        self.pub_cmd_vel.publish(cmd_msg)

        goal_handle.succeed()
        self.is_executing = False
        return result               # i don't assing anything to result

    async def trigger_unload(self):   # maybe return True if unloading successfull and false if not. Instead of raising an error 
        if not self.action_client.wait_for_server(timeout_sec=0.1):
            raise Exception('Server /unload_container not available')

        goal_msg = UnloadContainer.Goal()
        goal_handle = await self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )

        if not goal_handle.accepted:
            raise Exception('Goal rejected by server')

        result_response = await goal_handle.get_result_async()
        self.get_logger().info(f'Result: {result_response.result.message}')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'State: {feedback.current_state}')

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def drop_off_callback(self, msg):
        self.drop_off_distance = msg.ranges[0]

    async def async_sleep(self, duration_sec):
        future = Future()
        timer = self.create_timer(duration_sec, lambda: future.set_result(None))
        await future
        self.destroy_timer(timer)

def main(args=None):
    rclpy.init(args=args)
    node = DumpingExecutor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
