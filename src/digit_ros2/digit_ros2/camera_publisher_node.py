import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import socket
import struct


class DigitCameraPublisher(Node):
    def __init__(self):
        super().__init__('digit_camera_publisher')

        self.declare_parameter('device_index', 0)
        self.declare_parameter('stream_host', '')
        self.declare_parameter('stream_port', 8090)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('topic', 'digit/image_raw')
        self.declare_parameter('frame_id', 'digit_sensor')
        self.declare_parameter('publish_rate_hz', 30.0)

        device_index = self.get_parameter('device_index').value
        stream_host = self.get_parameter('stream_host').value
        stream_port = self.get_parameter('stream_port').value
        width = self.get_parameter('frame_width').value
        height = self.get_parameter('frame_height').value
        topic = self.get_parameter('topic').value
        self.frame_id = self.get_parameter('frame_id').value
        rate_hz = self.get_parameter('publish_rate_hz').value

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, topic, 10)

        self.cap = None
        self.sock = None

        if stream_host:
            self.get_logger().info(f'Connecting to network stream at {stream_host}:{stream_port}')
            self.sock = socket.create_connection((stream_host, stream_port), timeout=5.0)
            self.sock.setblocking(False)
            self._recv_buffer = b''
            self._awaiting_header = True
            self._pending_len = 0
        else:
            self.cap = cv2.VideoCapture(device_index)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            if not self.cap.isOpened():
                self.get_logger().error(f'Could not open camera at index {device_index}')
                raise RuntimeError('Failed to open DIGIT camera')

        self.get_logger().info(f'Publishing DIGIT frames on "{topic}" at {rate_hz} Hz')
        self.timer = self.create_timer(1.0 / rate_hz, self.publish_frame)

    def _poll_socket_for_latest_frame(self):
        """Non-blocking drain of the socket; returns the newest complete frame's
        bytes, or None if nothing new has arrived since the last call."""
        latest_frame_bytes = None

        while True:
            try:
                chunk = self.sock.recv(65536)
            except BlockingIOError:
                break
            except (ConnectionError, OSError) as e:
                self.get_logger().error(f'Network stream receive error: {e}')
                break

            if not chunk:
                self.get_logger().error('Stream connection closed')
                break

            self._recv_buffer += chunk

            while True:
                if self._awaiting_header:
                    if len(self._recv_buffer) < 4:
                        break
                    (self._pending_len,) = struct.unpack('>I', self._recv_buffer[:4])
                    self._recv_buffer = self._recv_buffer[4:]
                    self._awaiting_header = False
                else:
                    if len(self._recv_buffer) < self._pending_len:
                        break
                    latest_frame_bytes = self._recv_buffer[:self._pending_len]
                    self._recv_buffer = self._recv_buffer[self._pending_len:]
                    self._awaiting_header = True

        return latest_frame_bytes

    def publish_frame(self):
        if self.cap is not None:
            ok, frame = self.cap.read()
            if not ok:
                self.get_logger().warn('Failed to read frame from DIGIT camera')
                return
        else:
            frame_bytes = self._poll_socket_for_latest_frame()
            if frame_bytes is None:
                return
            frame = cv2.imdecode(np.frombuffer(frame_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.publisher.publish(msg)

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        if self.sock is not None:
            self.sock.close()
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
