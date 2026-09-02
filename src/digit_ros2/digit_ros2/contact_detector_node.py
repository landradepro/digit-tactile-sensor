import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from digit_interfaces.msg import ContactInfo
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2


class ContactDetectorNode(Node):
    def __init__(self):
        super().__init__('contact_detector')

        self.declare_parameter('input_topic', 'digit/image_raw')
        self.declare_parameter('output_topic', 'digit/contact')
        self.declare_parameter('diff_threshold', 25)
        self.declare_parameter('min_contact_area', 200)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.diff_threshold = self.get_parameter('diff_threshold').value
        self.min_contact_area = self.get_parameter('min_contact_area').value

        self.bridge = CvBridge()
        self.reference_gray = None
        self.latest_frame = None

        self.publisher = self.create_publisher(ContactInfo, output_topic, 10)
        self.subscription = self.create_subscription(Image, input_topic, self.image_callback, 10)
        self.capture_service = self.create_service(
            Trigger, '~/capture_reference', self.capture_reference_callback)

        self.get_logger().info(f'Subscribed to "{input_topic}", publishing contact info on "{output_topic}"')
        self.get_logger().info('Call the ~/capture_reference service with nothing touching the gel to start')

    def to_gray_blurred(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return cv2.GaussianBlur(gray, (9, 9), 0)

    def capture_reference_callback(self, request, response):
        if self.latest_frame is None:
            response.success = False
            response.message = 'No frame received yet'
            return response
        self.reference_gray = self.to_gray_blurred(self.latest_frame)
        response.success = True
        response.message = 'Reference frame captured'
        self.get_logger().info('Reference frame captured')
        return response

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.latest_frame = frame

        info = ContactInfo()
        info.header = msg.header

        if self.reference_gray is None:
            info.touching = False
            self.publisher.publish(info)
            return

        gray = self.to_gray_blurred(frame)
        diff = cv2.absdiff(gray, self.reference_gray)
        _, mask = cv2.threshold(diff, self.diff_threshold, 255, cv2.THRESH_BINARY)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contact_contours = [c for c in contours if cv2.contourArea(c) >= self.min_contact_area]

        if contact_contours:
            largest = max(contact_contours, key=cv2.contourArea)
            (cx, cy), radius = cv2.minEnclosingCircle(largest)
            info.touching = True
            info.area_px = int(cv2.contourArea(largest))
            info.center_x_px = float(cx)
            info.center_y_px = float(cy)
            info.radius_px = float(radius)
        else:
            info.touching = False

        self.publisher.publish(info)


def main(args=None):
    rclpy.init(args=args)
    node = ContactDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()