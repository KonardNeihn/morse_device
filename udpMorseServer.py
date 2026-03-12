import socket
import threading
import struct
import time
from flask import Flask, jsonify

PORT = 6969
PACKET_STRUCT = struct.Struct("<BI")   # status + signal
PACKET_SIZE = PACKET_STRUCT.size

clients = {}
client_stats = {}
clients_lock = threading.Lock()


# ───────────────── TCP helper ─────────────────
def recv_exact(sock, size):

    data = b''

    while len(data) < size:

        chunk = sock.recv(size - len(data))

        if not chunk:
            return None

        data += chunk

    return data


# ───────────────── Client Thread ─────────────────
def handle_client(conn, addr):

    print("Client connected:", addr)

    with clients_lock:

        clients[conn] = addr

        client_stats[addr] = {
            "packets": 0,
            "jitter": 0,
            "last_arrival": None,
            "rtt": None
        }

    last_ping = None

    try:

        while True:

            data = recv_exact(conn, PACKET_SIZE)

            if data is None:
                break

            status, signal = PACKET_STRUCT.unpack(data)

            now = time.time()

            with clients_lock:

                stats = client_stats[addr]
                stats["packets"] += 1

                if stats["last_arrival"] is not None:

                    delta = now - stats["last_arrival"]
                    stats["jitter"] = 0.9 * stats["jitter"] + 0.1 * abs(delta)

                stats["last_arrival"] = now


            # ───── server check (echo) ─────
            if status == 1:

                conn.sendall(data)

                if last_ping is not None:
                    client_stats[addr]["rtt"] = now - last_ping

                last_ping = now
                continue


            # ───── broadcast ─────
            with clients_lock:

                for client in list(clients.keys()):

                    if client == conn:
                        continue

                    try:
                        client.sendall(data)

                    except:
                        pass

    except Exception as e:

        print("Client error:", addr, e)

    finally:

        print("Client disconnected:", addr)

        with clients_lock:

            if conn in clients:
                del clients[conn]

        conn.close()


# ───────────────── TCP Server ─────────────────
def tcp_server():

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    server.bind(("0.0.0.0", PORT))
    server.listen()

    print("TCP Morse server running on port", PORT)

    while True:

        conn, addr = server.accept()

        thread = threading.Thread(
            target=handle_client,
            args=(conn, addr),
            daemon=True
        )

        thread.start()


# ───────────────── Web Monitor ─────────────────
app = Flask(__name__)

@app.route("/")
def index():

    with clients_lock:

        return jsonify({

            str(addr): {
                "packets": stats["packets"],
                "jitter": round(stats["jitter"], 6),
                "rtt": stats["rtt"]
            }

            for addr, stats in client_stats.items()
        })


def run_web():

    app.run(host="0.0.0.0", port=8080)


# ───────────────── Start ─────────────────
threading.Thread(target=tcp_server, daemon=True).start()
threading.Thread(target=run_web, daemon=True).start()

while True:
    time.sleep(1)