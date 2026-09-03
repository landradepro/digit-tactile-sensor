import socket
import struct
import sys
import time

HOST = sys.argv[1] if len(sys.argv) > 1 else '192.168.1.161'
PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 8090


def recv_exact(sock, n):
    buf = b''
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError('Connection closed')
        buf += chunk
    return buf


def main():
    print(f"Connecting to {HOST}:{PORT}...")
    sock = socket.create_connection((HOST, PORT), timeout=5.0)
    print("Connected. Measuring frame arrival rate (Ctrl+C to stop)...")

    count = 0
    last_print = time.time()

    while True:
        header = recv_exact(sock, 4)
        (length,) = struct.unpack('>I', header)
        recv_exact(sock, length)
        count += 1

        now = time.time()
        if now - last_print >= 1.0:
            print(f"frames in last second: {count}")
            count = 0
            last_print = now


if __name__ == '__main__':
    main()
