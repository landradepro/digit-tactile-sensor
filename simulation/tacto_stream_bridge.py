import time
import logging
import socket
import struct

import hydra
import numpy as np
import pybullet as p
import cv2

import pybulletX as px
import tacto

from sawyer_gripper import SawyerGripper

log = logging.getLogger(__name__)

STREAM_PORT = 8091
COMMAND_PORT = 8092

# (gripper_width_m, grip_force) per keyword. CLOSE/HOLD share a target: compliant
# position control naturally "holds" once it presses up against the object at the
# given force limit, so no separate hold-position logic is needed.
COMMAND_TO_GRIPPER = {
    'OPEN': (0.11, 20.0),
    'CLOSE': (0.035, 20.0),
    'HOLD': (0.035, 20.0),
    'REGRASP': (0.03, 40.0),
}


def start_server(port):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('0.0.0.0', port))
    server.listen(1)
    server.setblocking(False)
    return server


def to_uint8_bgr(frame):
    frame = np.asarray(frame)
    if frame.dtype != np.uint8:
        if frame.max() <= 1.0:
            frame = frame * 255.0
        frame = np.clip(frame, 0, 255).astype(np.uint8)
    return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)


def set_gripper_width(robot, width, grip_force):
    # Use the robot's fixed rest pose for the arm joints, not their current
    # (slightly noisy) live position - re-reading and re-commanding current
    # state on every call creates a feedback loop that shows up as shaking.
    joint_position = np.array(robot.zero_pose, dtype=float)
    joint_position[robot.gripper_joint_ids[0]] = -width / 2
    joint_position[robot.gripper_joint_ids[1]] = width / 2

    max_forces = np.ones(robot.num_dofs) * robot.MAX_FORCES
    max_forces[robot.gripper_joint_ids] = grip_force

    robot.set_joint_position(joint_position, max_forces, use_joint_effort_limits=False)


@hydra.main(config_path="conf", config_name="grasp")
def main(cfg):
    digits = tacto.Sensor(**cfg.tacto)

    log.info("Initializing world")
    px.init()

    p.resetDebugVisualizerCamera(**cfg.pybullet_camera)

    robot = SawyerGripper(**cfg.sawyer_gripper)

    digits.add_camera(robot.id, robot.digit_links)

    obj = px.Body(**cfg.object)
    digits.add_body(obj)

    np.set_printoptions(suppress=True)

    t = px.utils.SimulationThread(real_time_factor=1.0)
    t.start()

    robot.reset()

    panel = px.gui.RobotControlPanel(robot)
    panel.start()

    stream_server = start_server(STREAM_PORT)
    command_server = start_server(COMMAND_PORT)
    print(f"Streaming simulated DIGIT tactile feed on port {STREAM_PORT}")
    print(f"Listening for gripper commands on port {COMMAND_PORT}")
    stream_conn = None
    command_conn = None
    command_buffer = ''
    last_applied_command = None

    while True:
        color, depth = digits.render()
        digits.updateGUI(color, depth)

        if stream_conn is None:
            try:
                stream_conn, addr = stream_server.accept()
                print(f"ROS2 tactile client connected: {addr}")
            except BlockingIOError:
                pass

        if stream_conn is not None:
            frame = color[0] if isinstance(color, (list, tuple)) else color
            frame_bgr = to_uint8_bgr(frame)
            ok, jpeg = cv2.imencode('.jpg', frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                data = jpeg.tobytes()
                header = struct.pack('>I', len(data))
                try:
                    stream_conn.sendall(header + data)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    stream_conn = None

        if command_conn is None:
            try:
                command_conn, addr = command_server.accept()
                command_conn.setblocking(False)
                print(f"ROS2 gripper-command client connected: {addr}")
            except BlockingIOError:
                pass

        if command_conn is not None:
            try:
                chunk = command_conn.recv(1024)
                if not chunk:
                    command_conn = None
                else:
                    command_buffer += chunk.decode()
                    while '\n' in command_buffer:
                        line, command_buffer = command_buffer.split('\n', 1)
                        line = line.strip()
                        if line in COMMAND_TO_GRIPPER and line != last_applied_command:
                            width, force = COMMAND_TO_GRIPPER[line]
                            set_gripper_width(robot, width, force)
                            last_applied_command = line
            except BlockingIOError:
                pass
            except (ConnectionError, OSError):
                command_conn = None

        time.sleep(0.1)


if __name__ == "__main__":
    main()
