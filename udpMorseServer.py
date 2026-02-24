import socket
import threading
import queue
import time
import struct
import json
from flask import Flask, jsonify

INACTIVITY_TIMEOUT = 3600
RECEIVE_PORT = 6969

PACKET_STRUCT = struct.Struct("<B B H B B H B")

messages = queue.Queue()
clients = {}
last_seq = {}
client_stats = {}
clients_lock = threading.Lock()

# ─────────────────────────────
# JSON Logging
# ─────────────────────────────

def log_event(level, event, **kwargs):
    entry = {
        "timestamp": time.time(),
        "level": level,
        "event": event,
        **kwargs
    }
    with open("morse_server.jsonl", "a") as f:
        f.write(json.dumps(entry) + "\n")

# ─────────────────────────────
# Auto IP Bind
# ─────────────────────────────

server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server.bind(("0.0.0.0", RECEIVE_PORT))

print(f"Server running on UDP *:{RECEIVE_PORT}")
log_event("INFO", "server_start", port=RECEIVE_PORT)

# ─────────────────────────────

def receive():
    while True:
        message, address = server.recvfrom(64)

        now = time.time()

        if len(message) != PACKET_STRUCT.size:
            log_event("WARN", "invalid_packet", client=str(address))
            continue

        messages.put((message, address, now))

        with clients_lock:
            if address not in clients:
                log_event("INFO", "client_registered", client=str(address))
                client_stats[address] = {
                    "packets": 0,
                    "lost": 0,
                    "last_arrival": None,
                    "jitter": 0,
                }

            clients[address] = now


def broadcast():
    while True:
        message, from_address, arrival_time = messages.get()

        (
            session,
            cur_seq, cur_duration, cur_state,
            rec_seq, rec_duration, rec_state
        ) = PACKET_STRUCT.unpack(message)

        with clients_lock:

            stats = client_stats[from_address]
            stats["packets"] += 1

            # ───── Packet Loss ─────
            if from_address in last_seq:
                expected = (last_seq[from_address] + 1) & 0xFF
                if cur_seq != expected:
                    lost = (cur_seq - expected) & 0xFF
                    stats["lost"] += lost
                    log_event("WARN", "packet_loss",
                              client=str(from_address),
                              lost=lost)

            last_seq[from_address] = cur_seq

            # ───── Jitter ─────
            if stats["last_arrival"] is not None:
                delta = arrival_time - stats["last_arrival"]
                stats["jitter"] = 0.9 * stats["jitter"] + 0.1 * abs(delta)

            stats["last_arrival"] = arrival_time

            # ───── Broadcast ─────
            for client in list(clients.keys()):

                if time.time() - clients[client] > INACTIVITY_TIMEOUT:
                    log_event("INFO", "client_timeout",
                              client=str(client))
                    del clients[client]
                    continue

                if session == 0:
                    if client == from_address:
                        server.sendto(message, client)
                    continue

                if client != from_address:
                    server.sendto(message, client)

# ─────────────────────────────
# Web Monitor
# ─────────────────────────────

app = Flask(__name__)

@app.route("/")
def index():
    with clients_lock:
        return jsonify({
            "clients": {
                str(addr): {
                    "packets": stats["packets"],
                    "lost": stats["lost"],
                    "jitter": round(stats["jitter"], 6)
                }
                for addr, stats in client_stats.items()
            }
        })

def run_web():
    app.run(host="0.0.0.0", port=8080)

# ─────────────────────────────

t1 = threading.Thread(target=receive, daemon=True)
t2 = threading.Thread(target=broadcast, daemon=True)
t3 = threading.Thread(target=run_web, daemon=True)

t1.start()
t2.start()
t3.start()

while True:
    time.sleep(1)
