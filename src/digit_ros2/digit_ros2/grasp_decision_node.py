import rclpy
from rclpy.node import Node
from digit_interfaces.msg import ContactInfo, PressureEstimate
from std_msgs.msg import String
from collections import deque
from enum import Enum


class GripperState(Enum):
    IDLE = 'IDLE'
    CLOSING = 'CLOSING'
    HOLDING = 'HOLDING'
    SLIP_DETECTED = 'SLIP_DETECTED'
    REGRASPING = 'REGRASPING'


COMMAND_MAP = {
    GripperState.IDLE: 'OPEN',
    GripperState.CLOSING: 'CLOSE (increasing grip force)',
    GripperState.HOLDING: 'HOLD (target grip force reached)',
    GripperState.SLIP_DETECTED: 'ALERT: SLIP DETECTED',
    GripperState.REGRASPING: 'REGRASP (increasing force to recover grip)',
}


class GraspDecisionNode(Node):
    def __init__(self):
        super().__init__('grasp_decision')

        self.declare_parameter('contact_topic', 'digit/contact')
        self.declare_parameter('pressure_topic', 'digit/pressure')
        self.declare_parameter('command_topic', 'digit/gripper_command')
        self.declare_parameter('target_pressure', 350.0)
        self.declare_parameter('slip_drop_ratio', 0.3)
        self.declare_parameter('history_size', 5)

        contact_topic = self.get_parameter('contact_topic').value
        pressure_topic = self.get_parameter('pressure_topic').value
        command_topic = self.get_parameter('command_topic').value
        self.target_pressure = self.get_parameter('target_pressure').value
        self.slip_drop_ratio = self.get_parameter('slip_drop_ratio').value
        history_size = self.get_parameter('history_size').value

        self.pressure_history = deque(maxlen=history_size)
        self.state = GripperState.IDLE
        self.touching = False

        self.command_publisher = self.create_publisher(String, command_topic, 10)
        self.create_subscription(ContactInfo, contact_topic, self.contact_callback, 10)
        self.create_subscription(PressureEstimate, pressure_topic, self.pressure_callback, 10)

        self.get_logger().info(
            'Grasp decision node started - simulates what a real gripper controller '
            'would command based on live tactile feedback. No physical robot required; '
            'press on the gel to drive the state machine. Watch transitions below.')

    def contact_callback(self, msg):
        self.touching = msg.touching

    def pressure_callback(self, msg):
        if not self.touching:
            self._transition(GripperState.IDLE)
            self.pressure_history.clear()
            return

        pressure = msg.pressure_proxy
        self.pressure_history.append(pressure)

        if self.state in (GripperState.HOLDING, GripperState.REGRASPING):
            if self._detect_slip():
                self._transition(GripperState.SLIP_DETECTED)
                self._transition(GripperState.REGRASPING)
                return

        if pressure >= self.target_pressure:
            self._transition(GripperState.HOLDING)
        else:
            self._transition(GripperState.CLOSING)

    def _detect_slip(self):
        if len(self.pressure_history) < self.pressure_history.maxlen:
            return False
        oldest = self.pressure_history[0]
        newest = self.pressure_history[-1]
        if oldest <= 0:
            return False
        drop_ratio = (oldest - newest) / oldest
        return drop_ratio >= self.slip_drop_ratio

    def _transition(self, new_state):
        if new_state != self.state:
            self.get_logger().info(f'{self.state.value} -> {new_state.value}')
            self.state = new_state

        msg = String()
        msg.data = COMMAND_MAP[new_state]
        self.command_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GraspDecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
