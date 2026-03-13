import socket
import threading
import struct
import time

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

            # server test → echo zurück
            if status == 1:
                conn.sendall(data)
                continue

            # an alle anderen clients weiterleiten
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


def ping_loop():

    packet = PACKET.pack(2, 0)

    while True:

        time.sleep(10)

        with lock:
            dead = []

            for c in clients:
                try:
                    c.sendall(packet)
                except:
                    dead.append(c)

            for c in dead:
                if c in clients:
                    clients.remove(c)


def main():

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(("0.0.0.0", PORT))
    server.listen()

    print("Server running on port", PORT)

    # Ping thread starten
    threading.Thread(target=ping_loop, daemon=True).start()

    while True:

        conn, addr = server.accept()

        threading.Thread(
            target=handle_client,
            args=(conn,),
            daemon=True
        ).start()


main()