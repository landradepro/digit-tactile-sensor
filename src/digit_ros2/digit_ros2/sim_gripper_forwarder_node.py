import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket


class SimGripperCommandForwarder(Node):
    def __init__(self):
        super().__init__('sim_gripper_forwarder')

        self.declare_parameter('command_topic', 'digit/gripper_command')
        self.declare_parameter('sim_host', '127.0.0.1')
        self.declare_parameter('sim_port', 8092)

        command_topic = self.get_parameter('command_topic').value
        self.sim_host = self.get_parameter('sim_host').value
        self.sim_port = self.get_parameter('sim_port').value

        self.sock = None
        self.create_subscription(String, command_topic, self.command_callback, 10)
        self.get_logger().info(
            f'Forwarding gripper commands to TACTO sim at {self.sim_host}:{self.sim_port}')

    def _ensure_connected(self):
        if self.sock is not None:
            return True
        try:
            self.sock = socket.create_connection((self.sim_host, self.sim_port), timeout=1.0)
            return True
        except OSError:
            self.sock = None
            return False

    def command_callback(self, msg):
        if msg.data.startswith('OPEN'):
            keyword = 'OPEN'
        elif msg.data.startswith('CLOSE'):
            keyword = 'CLOSE'
        elif msg.data.startswith('HOLD'):
            keyword = 'HOLD'
        elif msg.data.startswith('REGRASP'):
            keyword = 'REGRASP'
        else:
            return

        if not self._ensure_connected():
            return

        try:
            self.sock.sendall((keyword + '\n').encode())
        except OSError:
            self.sock = None


def main(args=None):
    rclpy.init(args=args)
    node = SimGripperCommandForwarder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.sock is not None:
            node.sock.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
