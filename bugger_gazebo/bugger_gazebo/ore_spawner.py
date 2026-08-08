import math
import uuid
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from ros_gz_interfaces.srv import SpawnEntity

SPHERE_SDF = """
<sdf version="1.6">
  <model name="{model_name}">
    <pose>0 0 0 0 0 0</pose>
    <link name="link">
      <inertial>
        <mass>0.1</mass>
        <inertia><ixx>0.0001</ixx><iyy>0.0001</iyy><izz>0.0001</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <collision name="collision">
        <geometry><sphere><radius>0.02</radius></sphere></geometry>
      </collision>
      <visual name="visual">
        <geometry><sphere><radius>0.02</radius></sphere></geometry>
        <material><ambient>0.4 0.25 0.1 1</ambient></material>
      </visual>
    </link>
  </model>
</sdf>
"""
                                                                        
class OreSpawner(Node):
    def __init__(self):
        super().__init__('ore_spawner')

        self.sub_wheel = self.create_subscription(Float64, '/wheel_cmd', self.whell_callback, 10)
        self.sub_sensor = self.create_subscription(LaserScan, '/wheel_clearance/points', self.sensor_callback, 10)

        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')

        self.wheel_speed = 0.0
        self.min_distance = float('inf')

        self.mining_threshold_dist = 0.25
        self.wheel_min_speed = 0.1

        self.spawn_x = -0.3
        self.spawn_y = 0.0
        self.spawn_z = 0.4

        self.timer = self.create_timer(0.5, self.spawning_loop)
        self.get_logger().info('Ore spawner node initialized.')

    def whell_callback(self, msg):
        self.wheel_speed = msg.data

    def sensor_callback(self, msg):
        valid_ranges = [r for r in msg.ranges if not math.isinf(r) and not math.isnan(r)]
        if valid_ranges:
            self.min_distance = min(valid_ranges)
        else:
            self.min_distance = float('inf')

    def spawning_loop(self):
        if abs(self.wheel_speed) > self.wheel_min_speed and self.min_distance < self.mining_threshold_dist:
            self.spawn_ore()

    def spawn_ore(self):
        if not self.spawn_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().info('Gazebo spawn service (/spawn_entity) not available.')
            return

        request = SpawnEntity.Request()
        unique_name = f"ore_sphere_{uuid.uuid4().hex[:8]}"
        request.entity_factory.name = unique_name
        request.entity_factory.sdf = SPHERE_SDF.replace("{model_name}", unique_name)

        request.entity_factory.pose.position.x = self.spawn_x
        request.entity_factory.pose.position.y = self.spawn_y
        request.entity_factory.pose.position.z = self.spawn_z

        future = self.spawn_client.call_async(request)
        future.add_done_callback(self.spawn_callback)

    def spawn_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Ore sphere spawned successfully.')
        except Exception as e:
            self.get_logger().error(f'Failed to spawn ore: {e}')

def main(args=Node):
    rclpy.init(args=args)
    node = OreSpawner()
    try:                                        #looks weird
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()