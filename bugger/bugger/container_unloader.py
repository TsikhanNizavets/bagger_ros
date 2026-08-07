import time
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64
from bugger_interfaces.action import UnloadContainer       

class ContainerUnloader(Node):                                                                 
    def __init__(self):
        super().__init__('container_unloader')

        self.pub_container = self.create_publisher(Float64, '/container_cmd', 10)

        self._action_server = ActionServer(
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

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing unloading action...')

        feedback_msg = UnloadContainer.Feedback()
        result = UnloadContainer.Result()
        target_angle = goal_handle.request.dump_angle

        feedback_msg.current_state = 'TILTING'
        feedback_msg.current_angle = target_angle
        goal_handle.publish_feedback(feedback_msg)

        msg = Float64()
        msg.data = target_angle
        self.pub_container.publish(msg)

        feedback_msg.current_state = 'DUMPING'
        goal_handle.publish_feedback(feedback_msg)

        dump_time_sec = 10.0
        start_time = time.time()

        while (time.time() - start_time) < dump_time_sec:
            if goal_handle.is_cancel_requested:
                msg.data = self.home_angle
                self.pub_container.publish(msg)

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
        time.sleep(1.0)

        goal_handle.succeed()
        result.success = True
        result.message = 'Unloading finished successfully.'
        return result

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