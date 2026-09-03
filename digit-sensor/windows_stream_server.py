import argparse
import cv2
import socket
import struct
import threading

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
JPEG_QUALITY = 80

cap = None
capture_lock = threading.Lock()


def handle_client(conn, addr):
    print(f"Client connected: {addr}")
    try:
        while True:
            with capture_lock:
                ok, frame = cap.read()
            if not ok:
                continue

            ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            if not ok:
                continue

            data = jpeg.tobytes()
            header = struct.pack('>I', len(data))
            conn.sendall(header + data)
    except (BrokenPipeError, ConnectionResetError, OSError):
        pass
    finally:
        conn.close()
        print(f"Client disconnected: {addr}")


def main():
    global cap

    parser = argparse.ArgumentParser(description="Stream the DIGIT sensor over TCP to a Linux/ROS2 client")
    parser.add_argument('--device-index', type=int, default=0,
                         help='Camera device index for the DIGIT sensor (default: 0)')
    parser.add_argument('--port', type=int, default=8090,
                         help='TCP port to listen on (default: 8090)')
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.device_index, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    if not cap.isOpened():
        raise RuntimeError("Could not open DIGIT sensor")

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('0.0.0.0', args.port))
    server.listen(1)
    print(f"Streaming DIGIT sensor on port {args.port}. On the Linux side, use this PC's IP and this port.")
    print("Press Ctrl+C to stop")

    try:
        while True:
            conn, addr = server.accept()
            threading.Thread(target=handle_client, args=(conn, addr), daemon=True).start()
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        server.close()


if __name__ == '__main__':
    main()
