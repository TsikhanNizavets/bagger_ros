'''
Stops bucket wheel while unloading due to highest priority.

TODO: Handle the feedback properly. You send nonsense for both current_angle and current_state
'''

import time
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64
from bugger_interfaces.action import UnloadContainer       
from sensor_msgs.msg import JointState

class ContainerUnloader(Node):                                                                 
    def __init__(self):
        super().__init__('container_unloader')

        self.pub_container = self.create_publisher(Float64, '/container_cmd', 10)
        
        # Check if the bucket wheel has stopped
        self.wheel_speed = 0.0
        self.sub_join_states = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        # Publish zero velocity to the wheel to block it inside of timer, which starts in callback
        self.pub_cmd_wheel = self.create_publisher(Float64, '/bucket_wheel_cmd_priority', 10)
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
        self.get_logger().info('Action Server ready on /unload_container')

    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request!')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing unloading action...')

        self.block_wheel_timer = self.create_timer(0.1, self.timer_block_wheel)

        feedback_msg = UnloadContainer.Feedback()
        result = UnloadContainer.Result()
        target_angle = goal_handle.request.dump_angle

        # Wait untill the bucket wheel stops
        feedback_msg.current_state = 'WAITING_FOR_WHEEL_TO_STOP'
        goal_handle.publish_feedback(feedback_msg)

        while abs(self.wheel_speed) > 0.05:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = 'Unloading canceled while waiting for wheel to stop.'
                self.block_wheel_timer.cancel()
                return result
            time.sleep(0.1)

        feedback_msg.current_state = 'TILTING'
        feedback_msg.current_angle = target_angle       # you should return current angle, not target angle
        goal_handle.publish_feedback(feedback_msg)

        msg = Float64()
        msg.data = target_angle
        self.pub_container.publish(msg)

        feedback_msg.current_state = 'DUMPING'      # Dumping starts immediately after Tilting
        goal_handle.publish_feedback(feedback_msg)  # Check if the target_angle is reached before Dumping?

        dump_time_sec = 5.0
        start_time = time.time()

        while (time.time() - start_time) < dump_time_sec:
            if goal_handle.is_cancel_requested:
                msg.data = self.home_angle
                self.pub_container.publish(msg)
                time.sleep(3.5)                     # time for container to reach init pose, before unblock bucket wheel
                self.block_wheel_timer.cancel()
                goal_handle.canceled()
                result.success = False
                result.message = 'Unloading canceled mid-operation.'
                return result

            time.sleep(0.1)

        feedback_msg.current_state = 'RETURNING'
        feedback_msg.current_angle = self.home_angle
        goal_handle.publish_feedback(feedback_msg)

        msg.data = self.home_angle
        self.pub_container.publish(msg)
        time.sleep(3.5)         # i don't like this timer

        self.block_wheel_timer.cancel()
        goal_handle.succeed()
        result.success = True
        result.message = 'Unloading finished successfully.'
        return result

    def joint_state_callback(self, msg):
        if 'bucket_wheel_joint' in msg.name:
            idx = msg.name.index('bucket_wheel_joint')
            self.wheel_speed = msg.velocity[idx]

    def timer_block_wheel(self):
        msg = Float64()
        msg.data = 0.0
        self.pub_cmd_wheel.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ContainerUnloader()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()