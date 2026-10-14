from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseStamped, TransformStamped
from scipy.spatial.transform import Rotation
from tf2_ros import TransformBroadcaster

from scratchpad.cube_board_loader import load_cube_boards


class CubeTracker(Node):
    def __init__(self):
        super().__init__('cube_tracker')

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.distortion = None

        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_APRILTAG_36h11
        )

        package_directory = Path(get_package_share_directory('scratchpad'))

        self.boards = load_cube_boards(
            package_directory / 'config' / 'cube_boards_3d.yaml', 
            self.dictionary,
        )

        self.pose_publishers = {
            name: self.create_publisher(
                PoseStamped,
                f'/scratchpad/{name}/pose',
                10,
            )
            for name in self.boards
        }

        self.tf_broadcaster = TransformBroadcaster(self)

        self.info_subscription = self.create_subscription(
            CameraInfo,
            '/scratchpad/camera/camera_info',
            self.camera_info_callback,
            qos_profile_sensor_data,
        )

        self.image_subscription = self.create_subscription(
            Image,
            '/scratchpad/camera/image_raw',
            self.image_callback,
            qos_profile_sensor_data,
        )

    def camera_info_callback(self, message):
        matrix = np.array(message.k, dtype=np.float64).reshape(3, 3)

        if matrix[0, 0] <= 0 or matrix[1, 1] <= 0:
            return

        self.camera_matrix = matrix
        self.distortion = np.array(message.d, dtype=np.float64)

    def image_callback(self, message):
        if self.camera_matrix is None or not message.header.frame_id:
            return

        image = self.bridge.imgmsg_to_cv2(
            message, desired_encoding='bgr8'
        )

        corners, ids, _ = cv2.aruco.detectMarkers(
            image, self.dictionary
        )

        if ids is None:
            return

        for name, board in self.boards.items():
            count, rvec, tvec = cv2.aruco.estimatePoseBoard(
                corners,
                ids,
                board,
                self.camera_matrix,
                self.distortion,
                None,
                None,
            )

            if count == 0:
                continue

            self.publish_cube_pose(
                name, rvec, tvec, message.header
            )

            x, y, z = tvec.ravel()
            rotation = rvec.ravel().round(3).tolist()

            self.get_logger().info(
                f'{name}: {count} tags, '
                f'position=({x:.6f}, {y:.6f}, {z:.6f}) m, '
                f'rotation_vector={rotation}, '
                f'frame={message.header.frame_id}',
                throttle_duration_sec=1.0,
            )

    def publish_cube_pose(self, name, rvec, tvec, header):
        quaternion = Rotation.from_rotvec(rvec.ravel()).as_quat()

        pose = PoseStamped()
        pose.header = header

        (
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
        ) = map(float, tvec.ravel())

        (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ) = map(float, quaternion)

        self.pose_publishers[name].publish(pose)

        transform = TransformStamped()
        transform.header = header
        transform.child_frame_id = f'board_{name}'

        transform.transform.translation.x = pose.pose.position.x
        transform.transform.translation.y = pose.pose.position.y
        transform.transform.translation.z = pose.pose.position.z
        transform.transform.rotation = pose.pose.orientation

        self.tf_broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = CubeTracker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
