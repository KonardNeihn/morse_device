import socket
import threading
import struct

PORT = 6969
PACKET = struct.Struct("<BI")
SIZE = PACKET.size

clients = []
lock = threading.Lock()


def recv_exact(sock, size):

    data = b''

    while len(data) < size:

        chunk = sock.recv(size - len(data))

        if not chunk:
            return None

        data += chunk

    return data


def handle_client(conn):

    print("Client connected")

    with lock:
        clients.append(conn)

    try:

        while True:

            data = recv_exact(conn, SIZE)

            if data is None:
                break

            status, signal = PACKET.unpack(data)

            if status == 1:
                conn.sendall(data)
                continue

            with lock:
                for c in clients:
                    if c != conn:
                        try:
                            c.sendall(data)
                        except:
                            pass

    finally:

        print("Client disconnected")

        with lock:
            if conn in clients:
                clients.remove(conn)

        conn.close()


def main():

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(("0.0.0.0", PORT))
    server.listen()

    print("Server running on port", PORT)

    while True:

        conn, addr = server.accept()

        threading.Thread(
            target=handle_client,
            args=(conn,),
            daemon=True
        ).start()


main()