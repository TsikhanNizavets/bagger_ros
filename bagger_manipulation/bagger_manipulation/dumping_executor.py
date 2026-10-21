"""
Action. Drives backward, when distance jumps, stops, unloads, and drives back in the safety zone
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
from action_msgs.msg import GoalStatus

class DumpingExecutor(Node):
    def __init__(self):
        super().__init__('dumping_executor')

        self.drop_off_distance = 0
        self.linear_speed = 0.1
        self.latest_odom = None
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
        self.is_executing = True
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request!')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        try:
            self.get_logger().info('Executing dumping action...')

            feedback_msg = ExecuteDumping.Feedback()
            result = ExecuteDumping.Result()

            feedback_msg.current_state = 'DOCKING'
            goal_handle.publish_feedback(feedback_msg)

            cmd_msg = TwistStamped()
            cmd_msg.header.frame_id = "base_link"

            start_x = self.latest_odom.pose.pose.position.x
            start_y = self.latest_odom.pose.pose.position.y
            driven_distance = 0.0

            while self.drop_off_distance < 0.7:
                if goal_handle.is_cancel_requested:
                    cmd_msg.header.stamp = self.get_clock().now().to_msg()
                    cmd_msg.twist.linear.x = 0.0
                    self.pub_cmd_vel.publish(cmd_msg)

                    while abs(self.latest_odom.twist.twist.linear.x) > 0.0001:
                        await self.async_sleep(0.1)

                    goal_handle.canceled()

                    result.message = 'Canceled while docking'
                    return result

                cmd_msg.header.stamp = self.get_clock().now().to_msg()
                cmd_msg.twist.linear.x = -self.linear_speed
                self.pub_cmd_vel.publish(cmd_msg)

                driven_distance = math.hypot(
                    self.latest_odom.pose.pose.position.x - start_x,
                    self.latest_odom.pose.pose.position.y - start_y
                )

                await self.async_sleep(0.1)

            cmd_msg.header.stamp = self.get_clock().now().to_msg()
            cmd_msg.twist.linear.x = 0.0
            self.pub_cmd_vel.publish(cmd_msg)

            feedback_msg.current_state = 'UNLOADING'
            goal_handle.publish_feedback(feedback_msg)

            success, message = await self.trigger_unload(goal_handle)

            if goal_handle.is_cancel_requested:
                result.message = f'Canceled during unloading: {message}'
                goal_handle.canceled()
                return result

            if not success:
                goal_handle.abort()

                result.message = message
                return result

            return_start_x = self.latest_odom.pose.pose.position.x
            return_start_y = self.latest_odom.pose.pose.position.y
            return_distance = 0.0

            while return_distance < driven_distance:
                if goal_handle.is_cancel_requested:
                    cmd_msg.header.stamp = self.get_clock().now().to_msg()
                    cmd_msg.twist.linear.x = 0.0
                    self.pub_cmd_vel.publish(cmd_msg)

                    while abs(self.latest_odom.twist.twist.linear.x) > 0.0001:
                        await self.async_sleep(0.1)

                    goal_handle.canceled()

                    result.message = 'Canceled while returning'
                    return result
                
                cmd_msg.header.stamp = self.get_clock().now().to_msg()
                cmd_msg.twist.linear.x = self.linear_speed
                self.pub_cmd_vel.publish(cmd_msg)

                return_distance = math.hypot(
                    return_start_x - self.latest_odom.pose.pose.position.x,
                    return_start_y - self.latest_odom.pose.pose.position.y
                )

                await self.async_sleep(0.1)

            cmd_msg.header.stamp = self.get_clock().now().to_msg()
            cmd_msg.twist.linear.x = 0.0
            self.pub_cmd_vel.publish(cmd_msg)

            goal_handle.succeed()
            return result
        
        except Exception as error:
            message = f'Dumping failed: {error}'
            self.get_logger().error(message)

            result = ExecuteDumping.Result()
            result.message = message

            return result

        finally:
            stop_msg = TwistStamped()
            stop_msg.header.frame_id = "base_link"
            stop_msg.header.stamp = self.get_clock().now().to_msg()
            stop_msg.twist.linear.x = 0.0

            self.pub_cmd_vel.publish(stop_msg)
            self.is_executing = False

    async def trigger_unload(self, parent_goal_handle):
        if not self.action_client.wait_for_server(timeout_sec=0.1):
            return False, 'Server /unload_container not available'

        child_goal_handle = await self.action_client.send_goal_async(
            UnloadContainer.Goal(),
            feedback_callback=lambda feedback_msg: self.feedback_callback(
                feedback_msg, parent_goal_handle
            )
        )

        if not child_goal_handle.accepted:
            return False, 'Goal rejected by server'

        result_future = child_goal_handle.get_result_async()
        cancel_sent = False

        while not result_future.done():
            if parent_goal_handle.is_cancel_requested and not cancel_sent:
                cancel_sent = True
                await child_goal_handle.cancel_goal_async()

            if not result_future.done():
                await self.async_sleep(0.1)

        result_response = result_future.result()

        success = result_response.status == GoalStatus.STATUS_SUCCEEDED
        return success, result_response.result.message

    def feedback_callback(self, feedback_msg, parent_goal_handle):
        feedback = ExecuteDumping.Feedback()

        feedback.current_state = f'DUMPING: {feedback_msg.feedback.current_state}'
        parent_goal_handle.publish_feedback(feedback)

    def odom_callback(self, msg):
        self.latest_odom = msg

    def drop_off_callback(self, msg):
        self.drop_off_distance = msg.ranges[0]

    async def async_sleep(self, duration_sec):
        future = Future()

        timer = self.create_timer(duration_sec, lambda: future.set_result(None))

        try:
            await future
        finally:
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
