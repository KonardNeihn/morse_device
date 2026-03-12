import socket
import threading
import struct
import time
import json
from flask import Flask, jsonify

# ───────────── Konfiguration ─────────────
PORT = 6969
INACTIVITY_TIMEOUT = 60

PACKET_STRUCT = struct.Struct("<BI")  # status + signal

clients = {}
client_stats = {}
clients_lock = threading.Lock()

# ───────────── Logging ─────────────
def log_event(level, event, **kwargs):

    entry = {
        "timestamp": time.time(),
        "level": level,
        "event": event,
        **kwargs
    }

    with open("morse_server.jsonl", "a") as f:
        f.write(json.dumps(entry) + "\n")


# ───────────── Client Handler ─────────────
def handle_client(conn, addr):

    print("Client connected:", addr)

    with clients_lock:
        clients[conn] = {
            "addr": addr,
            "last_seen": time.time()
        }

        client_stats[addr] = {
            "packets": 0,
            "lost": 0,
            "last_arrival": None,
            "jitter": 0,
            "rtt": None
        }

    last_signal = None

    try:

        while True:

            data = conn.recv(PACKET_STRUCT.size)

            if not data:
                break

            status, signal = PACKET_STRUCT.unpack(data)
            now = time.time()

            with clients_lock:

                stats = client_stats[addr]
                stats["packets"] += 1

                # jitter berechnen
                if stats["last_arrival"] is not None:
                    delta = now - stats["last_arrival"]
                    stats["jitter"] = 0.9 * stats["jitter"] + 0.1 * abs(delta)

                stats["last_arrival"] = now

            # ───── Server Check (Echo) ─────
            if status == 1:

                conn.sendall(data)

                # RTT schätzen
                if last_signal is not None:
                    rtt = now - last_signal
                    client_stats[addr]["rtt"] = rtt

                last_signal = now
                continue


            # ───── Broadcast ─────
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

        with clients_lock:
            if conn in clients:
                del clients[conn]

        conn.close()

        print("Client disconnected:", addr)


# ───────────── Server ─────────────
def tcp_server():

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    server.bind(("0.0.0.0", PORT))
    server.listen()

    print("TCP Morse Server running on port", PORT)

    while True:

        conn, addr = server.accept()

        thread = threading.Thread(
            target=handle_client,
            args=(conn, addr),
            daemon=True
        )

        thread.start()


# ───────────── Web Monitor ─────────────
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


# ───────────── Threads ─────────────
t1 = threading.Thread(target=tcp_server, daemon=True)
t2 = threading.Thread(target=run_web, daemon=True)

t1.start()
t2.start()

while True:
    time.sleep(1)