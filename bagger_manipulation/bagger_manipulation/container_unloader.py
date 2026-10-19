'''
Stops bucket wheel while unloading due to highest priority.
'''

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from std_msgs.msg import Float64
from control_msgs.msg import MultiDOFCommand
from bagger_interfaces.action import UnloadContainer       
from sensor_msgs.msg import JointState
from rclpy.task import Future

class ContainerUnloader(Node):                                                                 
    def __init__(self):
        super().__init__('container_unloader')

        self.pub_container = self.create_publisher(Float64, '/container_cmd', 10)
        
        # Check if the bucket wheel has stopped
        self.wheel_speed = 0.0
        self.container_position = 0.0
        self.sub_join_states = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        # Publish zero velocity to the wheel to block it inside of timer, which starts in callback
        self.pub_cmd_wheel = self.create_publisher(MultiDOFCommand, '/bucket_wheel_cmd_priority', 10)
        self.block_wheel_timer = None
                                                                                                
        self.action_server = ActionServer(
            self,
            UnloadContainer,
            'unload_container',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.home_angle = 0.0
        self.target_angle = -2.75
        self.dumping_duration = 3.0
        self.is_executing = False

        self.get_logger().info('Action Server ready on /unload_container')

    async def goal_callback(self, goal_request):
        if self.is_executing:
            return GoalResponse.REJECT
        self.is_executing = True
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request!')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        try:
            self.get_logger().info('Executing unloading action...')

            self.block_wheel_timer = self.create_timer(0.1, self.timer_block_wheel)

            feedback_msg = UnloadContainer.Feedback()
            feedback_msg.current_state = 'WAITING_FOR_WHEEL_TO_STOP'
            goal_handle.publish_feedback(feedback_msg)

            while abs(self.wheel_speed) > 0.05:
                if goal_handle.is_cancel_requested:
                    return await self.cancel_unloading(
                        goal_handle, 'Canceled while stopping wheel'
                    )
                await self.async_sleep(0.1)

            feedback_msg.current_state = 'TILTING CONTAINER'
            goal_handle.publish_feedback(feedback_msg)

            msg = Float64()
            msg.data = self.target_angle
            self.pub_container.publish(msg)

            while abs(self.target_angle - self.container_position) > 0.04:
                if goal_handle.is_cancel_requested:
                    return await self.return_home_and_cancel_unloading(
                        goal_handle, 'Canceled while tilting container'
                    )
                await self.async_sleep(0.1)

            feedback_msg.current_state = 'DUMPING'
            goal_handle.publish_feedback(feedback_msg)

            start_time = self.get_clock().now()

            while (self.get_clock().now() - start_time).nanoseconds / 1_000_000_000 < self.dumping_duration:
                if goal_handle.is_cancel_requested:
                    return await self.return_home_and_cancel_unloading(
                        goal_handle, 'Canceled while dumping'
                    )
                await self.async_sleep(0.1)

            feedback_msg.current_state = 'RETURNING CONTAINER'
            goal_handle.publish_feedback(feedback_msg)

            msg.data = self.home_angle
            self.pub_container.publish(msg)

            while abs(self.home_angle - self.container_position) > 0.04:
                if goal_handle.is_cancel_requested:
                    return await self.cancel_unloading(
                        goal_handle, 'Canceled while returning container'
                    )
                await self.async_sleep(0.1)

            self.block_wheel_timer.cancel()
            goal_handle.succeed()

            self.is_executing = False

            result = UnloadContainer.Result()
            result.message = 'FINISHED SUCCESSFULLY'
            return result

        except Exception as error:
            message = f'Unloading failed: {error}'
            self.get_logger().error(message)

            result = UnloadContainer.Result()
            result.message = message

            return result

        finally:
            msg = Float64()
            msg.data = self.home_angle
            self.pub_container.publish(msg)
            
            while abs(self.home_angle - self.container_position) > 0.04:
                await self.async_sleep(0.1)

            self.block_wheel_timer.cancel()

            self.is_executing = False

    async def return_home_and_cancel_unloading(self, goal_handle, message):
        msg = Float64()
        msg.data = self.home_angle
        self.pub_container.publish(msg)

        return await self.cancel_unloading(goal_handle, message)

    async def cancel_unloading(self, goal_handle, message):
        feedback_msg = UnloadContainer.Feedback()
        feedback_msg.current_state = 'CANCELING'
        goal_handle.publish_feedback(feedback_msg)
        
        while abs(self.home_angle - self.container_position) > 0.04:
            await self.async_sleep(0.1)
        
        self.block_wheel_timer.cancel()
        goal_handle.canceled()

        self.is_executing = False

        result = UnloadContainer.Result()
        result.message = message
        return result
    
    async def async_sleep(self, duration_sec):
        future = Future()

        timer = self.create_timer(duration_sec, lambda: future.set_result(None))

        try:
            await future
        finally:
            self.destroy_timer(timer)

    def joint_state_callback(self, msg):
        if 'bucket_wheel_joint' in msg.name:
            idx = msg.name.index('bucket_wheel_joint')
            self.wheel_speed = msg.velocity[idx]
        if 'container_joint' in msg.name:
            idx = msg.name.index('container_joint')
            self.container_position = msg.position[idx]

    def timer_block_wheel(self):
        msg = MultiDOFCommand()
        msg.dof_names = ['bucket_wheel_joint']
        msg.values = [0.0]
        self.pub_cmd_wheel.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ContainerUnloader()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
