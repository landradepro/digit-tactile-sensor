import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class DigitCameraPublisher(Node):
    def __init__(self):
        super().__init__('digit_camera_publisher')

        self.declare_parameter('device_index', 0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('topic', 'digit/image_raw')
        self.declare_parameter('frame_id', 'digit_sensor')
        self.declare_parameter('publish_rate_hz', 30.0)

        device_index = self.get_parameter('device_index').value
        width = self.get_parameter('frame_width').value
        height = self.get_parameter('frame_height').value
        topic = self.get_parameter('topic').value
        self.frame_id = self.get_parameter('frame_id').value
        rate_hz = self.get_parameter('publish_rate_hz').value

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, topic, 10)

        self.cap = cv2.VideoCapture(device_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        if not self.cap.isOpened():
            self.get_logger().error(f'Could not open camera at index {device_index}')
            raise RuntimeError('Failed to open DIGIT camera')

        self.get_logger().info(f'Publishing DIGIT frames on "{topic}" at {rate_hz} Hz')
        self.timer = self.create_timer(1.0 / rate_hz, self.publish_frame)

    def publish_frame(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn('Failed to read frame from DIGIT camera')
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.publisher.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DigitCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()