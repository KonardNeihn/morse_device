import socket
import threading
import struct
import time

PORT = 6969

# Packet: uint8 status + uint32 signal = 5 bytes
PACKET = struct.Struct("<BI")
SIZE = PACKET.size  # should be 5

clients = []
lock = threading.Lock()


def recv_exact(sock, size):
    data = b''
    while len(data) < size:
        try:
            chunk = sock.recv(size - len(data))
        except socket.timeout:
            continue
        except Exception:
            return None

        if not chunk:
            return None

        data += chunk
    return data


def remove_client(conn):
    with lock:
        if conn in clients:
            clients.remove(conn)

    try:
        conn.close()
    except:
        pass


def safe_send(conn, data):
    try:
        conn.sendall(data)
        return True
    except Exception:
        return False


def broadcast(data, exclude=None):
    dead = []

    with lock:
        current_clients = list(clients)

    for c in current_clients:
        if c == exclude:
            continue

        if not safe_send(c, data):
            dead.append(c)

    for c in dead:
        print("Removing dead client during broadcast")
        remove_client(c)


def handle_client(conn, addr):
    print(f"Client connected: {addr}")

    # Important: disable Nagle on this connection
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    # Prevent recv from blocking forever
    conn.settimeout(1.0)

    with lock:
        clients.append(conn)

    try:
        while True:
            data = recv_exact(conn, SIZE)
            if data is None:
                break

            try:
                status, signal = PACKET.unpack(data)
            except Exception:
                print(f"Bad packet from {addr}")
                break

            # status = 1 -> echo back only to sender
            if status == 1:
                if not safe_send(conn, data):
                    break
                continue

            # status = 0 (normal) or anything else -> broadcast to others
            broadcast(data, exclude=conn)

    finally:
        print(f"Client disconnected: {addr}")
        remove_client(conn)


def ping_thread():
    while True:
        time.sleep(10)

        # status = 2, signal dummy = 0
        ping_packet = PACKET.pack(2, 0)

        with lock:
            current_clients = list(clients)

        if current_clients:
            print(f"Sending ping to {len(current_clients)} clients")

        dead = []
        for c in current_clients:
            if not safe_send(c, ping_packet):
                dead.append(c)

        for c in dead:
            print("Removing dead client during ping")
            remove_client(c)


def main():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Optional but useful for quick restart after crash
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    # Not super important on listening socket, but harmless
    server.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    server.bind(("0.0.0.0", PORT))
    server.listen()

    print(f"Server running on port {PORT} (packet size = {SIZE} bytes)")

    # Start ping thread
    threading.Thread(target=ping_thread, daemon=True).start()

    while True:
        conn, addr = server.accept()

        threading.Thread(
            target=handle_client,
            args=(conn, addr),
            daemon=True
        ).start()


if __name__ == "__main__":
    main()