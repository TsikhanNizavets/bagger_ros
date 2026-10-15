import cv2
import rclpy

from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class CornerInspector(Node):
    def __init__(self):
        super().__init__('corner_inspector')

        self.bridge = CvBridge()
        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_APRILTAG_36h11
        )

        self.publisher = self.create_publisher(
            Image, '/camera/tag_corners', 10
        )

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.inspect_image,
            qos_profile_sensor_data,
        )

    def inspect_image(self, message):
        image = self.bridge.imgmsg_to_cv2(
            message, desired_encoding='bgr8'
        )

        corners, ids, _ = cv2.aruco.detectMarkers(
            image, self.dictionary
        )

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(image, corners, ids)

            for marker_corners in corners:
                for index, point in enumerate(marker_corners[0]):
                    x, y = map(int, point)

                    # OpenCV colours use blue, green, red order.
                    colour = (0, 0, 255) if index == 0 else (0, 255, 255)

                    cv2.circle(image, (x, y), 3, colour, -1)
                    cv2.putText(
                        image,
                        f'c{index}',
                        (x + 5, y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.45,
                        colour,
                        1,
                        cv2.LINE_AA,
                    )

        annotated = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        annotated.header = message.header
        self.publisher.publish(annotated)


def main():
    rclpy.init()
    node = CornerInspector()

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
