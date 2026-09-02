import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import os
from scipy.spatial import cKDTree


class DepthReconstructorNode(Node):
    def __init__(self):
        super().__init__('depth_reconstructor')

        self.declare_parameter('input_topic', 'digit/image_raw')
        self.declare_parameter('output_topic', 'digit/depth')
        self.declare_parameter('calibration_dir', os.path.expanduser('~/digit_ws/calibration_data'))
        self.declare_parameter('diff_threshold', 25)
        self.declare_parameter('min_contact_area', 200)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        calibration_dir = self.get_parameter('calibration_dir').value
        self.diff_threshold = self.get_parameter('diff_threshold').value
        self.min_contact_area = self.get_parameter('min_contact_area').value

        with open(os.path.join(calibration_dir, 'lut_meta.json')) as f:
            meta = json.load(f)
        self.px_per_mm = meta['px_per_mm']
        self.pixel_spacing_mm = 1.0 / self.px_per_mm

        colors = np.load(os.path.join(calibration_dir, 'lut_colors.npy'))
        self.gradients = np.load(os.path.join(calibration_dir, 'lut_gradients.npy'))
        self.tree = cKDTree(colors)
        self.get_logger().info(f'Loaded lookup table with {len(colors)} entries from {calibration_dir}')

        reference_raw = cv2.imread(os.path.join(calibration_dir, 'reference.png'))
        if reference_raw is None:
            raise RuntimeError(f'Could not load reference.png from {calibration_dir}')
        self.reference_color = self._color_blur(reference_raw)
        self.reference_gray = self._gray_blur(reference_raw)

        self.bridge = CvBridge()
        self.depth_publisher = self.create_publisher(Image, output_topic, 10)
        self.peak_publisher = self.create_publisher(Float32, output_topic + '/peak_mm', 10)
        self.subscription = self.create_subscription(Image, input_topic, self.image_callback, 10)

        self.get_logger().info(f'Subscribed to "{input_topic}", publishing depth on "{output_topic}"')

    def _color_blur(self, frame):
        return cv2.GaussianBlur(frame, (5, 5), 0).astype(np.float32)

    def _gray_blur(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return cv2.GaussianBlur(gray, (9, 9), 0)

    def _integrate_gradients(self, gx, gy):
        h, w = gx.shape
        wx = np.fft.fftfreq(w, d=self.pixel_spacing_mm) * 2 * np.pi
        wy = np.fft.fftfreq(h, d=self.pixel_spacing_mm) * 2 * np.pi
        wx_grid, wy_grid = np.meshgrid(wx, wy)

        fx = np.fft.fft2(gx)
        fy = np.fft.fft2(gy)

        denom = wx_grid ** 2 + wy_grid ** 2
        denom[0, 0] = 1.0

        z_hat = (-1j * wx_grid * fx - 1j * wy_grid * fy) / denom
        z_hat[0, 0] = 0.0

        return np.real(np.fft.ifft2(z_hat))

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        gray = self._gray_blur(frame)
        gray_diff = cv2.absdiff(gray, self.reference_gray)
        _, mask = cv2.threshold(gray_diff, self.diff_threshold, 255, cv2.THRESH_BINARY)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contact_contours = [c for c in contours if cv2.contourArea(c) >= self.min_contact_area]

        gx = np.zeros(gray.shape, dtype=np.float32)
        gy = np.zeros(gray.shape, dtype=np.float32)
        peak_depth_mm = 0.0

        if contact_contours:
            contact_mask = np.zeros_like(mask)
            cv2.drawContours(contact_mask, contact_contours, -1, 255, -1)

            ys, xs = np.where(contact_mask == 255)
            color_diff = self._color_blur(frame) - self.reference_color
            query_colors = color_diff[ys, xs]

            _, idx = self.tree.query(query_colors, k=1)
            matched = self.gradients[idx]

            gx[ys, xs] = matched[:, 0]
            gy[ys, xs] = matched[:, 1]

            height_map = self._integrate_gradients(gx, gy)
            peak_depth_mm = float(-height_map.min())
        else:
            height_map = np.zeros(gray.shape, dtype=np.float64)

        depth_msg = self.bridge.cv2_to_imgmsg(height_map.astype(np.float32), encoding='32FC1')
        depth_msg.header = msg.header
        self.depth_publisher.publish(depth_msg)

        peak_msg = Float32()
        peak_msg.data = peak_depth_mm
        self.peak_publisher.publish(peak_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthReconstructorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()