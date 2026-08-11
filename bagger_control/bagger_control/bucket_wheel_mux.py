"""
Multiplex for bucket wheel topics

Handle namespaces for topics in future
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class BoolMuxNode(Node):

    def __init__(self):
        super().__init__(
            'bucket_wheel_mux',                   # name for node here and inside config file must match. namespaces later
            automatically_declare_parameters_from_overrides=True,
        )

        self.output_topic = (
            self.get_parameter('output_topic').value
            if self.has_parameter('output_topic')
            else 'cmd_bool_out'
        )
        self.default_value = (
            self.get_parameter('default_value').value
            if self.has_parameter('default_value')
            else False
        )
        rate = (
            self.get_parameter('publish_rate_hz').value
            if self.has_parameter('publish_rate_hz')
            else 20.0
        )

        self.sources = self._parse_sources()

        self.pub = self.create_publisher(Float64, self.output_topic, 10)
        self.topic_states = {}

        for src in self.sources:
            topic = src['topic']
            self.topic_states[topic] = {'val': None, 'last_time': None}
            self.create_subscription(
                Float64,
                topic,
                lambda msg, t=topic: self._input_callback(msg, t),
                10,
            )

        self.timer = self.create_timer(1.0 / rate, self._evaluate_and_publish)
        self.get_logger().info(
            f'[{self.get_name()}] Multiplexer initialized. Publishing to: "{self.output_topic}"'
        )

    def _parse_sources(self):
        # Extracts dynamic sources from ROS 2 YAML parameter file
        sources_params = self.get_parameters_by_prefix('sources')
        grouped = {}

        for key, param in sources_params.items():
            parts = key.split('.')
            if len(parts) == 2:
                src_name, field = parts[0], parts[1]
                if src_name not in grouped:
                    grouped[src_name] = {}
                grouped[src_name][field] = param.value

        sources_list = []
        for name, data in grouped.items():
            if 'topic' in data and 'priority' in data and 'timeout' in data:
                sources_list.append(
                    {
                        'name': name,
                        'topic': str(data['topic']),
                        'priority': int(data['priority']),
                        'timeout': float(data['timeout']),
                    }
                )

        if not sources_list:
            self.get_logger().fatal(
                f'[{self.get_name()}] Missing "sources" configuration in parameters! Shutting down.'
            )
            raise RuntimeError("Failed to initialize multiplexer: No source topics provided.")

        return sources_list

    def _input_callback(self, msg: Float64, topic_name: str):
        self.topic_states[topic_name]['val'] = msg.data
        self.topic_states[topic_name]['last_time'] = self.get_clock().now()

    def _evaluate_and_publish(self):
        now = self.get_clock().now()
        selected_value = self.default_value
        highest_priority = -1

        for src in self.sources:
            topic = src['topic']
            priority = src['priority']
            timeout = src['timeout']

            state = self.topic_states[topic]
            if state['last_time'] is not None:
                elapsed_sec = (now - state['last_time']).nanoseconds / 1e9
                if elapsed_sec <= timeout:
                    if priority > highest_priority:
                        highest_priority = priority
                        selected_value = state['val']

        out_msg = Float64()
        out_msg.data = selected_value
        self.pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BoolMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()