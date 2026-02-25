import socket
import threading
import queue
import time
import struct
import json
from flask import Flask, jsonify

# ────────────── Konfiguration ──────────────
INACTIVITY_TIMEOUT = 3600
RECEIVE_PORT = 6969
FRAMES_PER_BLOCK = 4  # muss zum ESP passen

# Packets: type, block_id, frame_id, timestamp, signal
PACKET_STRUCT = struct.Struct("<B B B I B")

messages = queue.Queue()
clients = {}
client_stats = {}
last_seq = {}
clients_lock = threading.Lock()

# ────────────── Logging ──────────────
def log_event(level, event, **kwargs):
    entry = {
        "timestamp": time.time(),
        "level": level,
        "event": event,
        **kwargs
    }
    with open("morse_server.jsonl", "a") as f:
        f.write(json.dumps(entry) + "\n")

# ────────────── Server Setup ──────────────
server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server.bind(("0.0.0.0", RECEIVE_PORT))
print(f"Server running on UDP *:{RECEIVE_PORT}")
log_event("INFO", "server_start", port=RECEIVE_PORT)

# ────────────── Empfang ──────────────
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
                    "fec_needed": 0,
                    "fec_failed": 0
                }
            clients[address] = now

# ────────────── Broadcast / FEC ──────────────
blocks = {}  # (client_addr, block_id) -> {"frames": [...], "mask": int, "fec": val}

def broadcast():
    while True:
        message, from_address, arrival_time = messages.get()
        type_, block_id, frame_id, timestamp, signal = PACKET_STRUCT.unpack(message)

        key = (from_address, block_id)

        # Block initialisieren
        if key not in blocks:
            blocks[key] = {
                "frames": [None] * FRAMES_PER_BLOCK,
                "mask": 0,
                "fec": None
            }

        block = blocks[key]

        # FEC oder DATA
        if type_ == 0:  # DATA
            block["frames"][frame_id] = signal
            block["mask"] |= (1 << frame_id)
        elif type_ == 1:  # FEC
            block["fec"] = signal

        # ───── Statistik Update ─────
        with clients_lock:
            stats = client_stats[from_address]
            stats["packets"] += 1

            if from_address in last_seq:
                expected = (last_seq[from_address] + 1) & 0xFF
                if frame_id != expected:
                    lost = (frame_id - expected) & 0xFF
                    stats["lost"] += lost
                    log_event("WARN", "packet_loss",
                              client=str(from_address),
                              lost=lost)
            last_seq[from_address] = frame_id

            # Jitter
            if stats["last_arrival"] is not None:
                delta = arrival_time - stats["last_arrival"]
                stats["jitter"] = 0.9 * stats["jitter"] + 0.1 * abs(delta)
            stats["last_arrival"] = arrival_time

        # ───── FEC Recovery ─────
        missing_count = block["frames"].count(None)
        if missing_count == 1 and block["fec"] is not None:
            missing_index = block["frames"].index(None)
            recovered = block["fec"]
            for i, val in enumerate(block["frames"]):
                if val is not None:
                    recovered ^= val
            block["frames"][missing_index] = recovered
            block["mask"] |= (1 << missing_index)
            with clients_lock:
                stats["fec_needed"] += 1
            log_event("INFO", "fec_recovered", client=str(from_address),
                      block=block_id, frame=missing_index)

        elif missing_count > 1 and block["fec"] is not None:
            with clients_lock:
                stats["fec_failed"] += 1
            log_event("WARN", "fec_failed", client=str(from_address),
                      block=block_id, missing=missing_count)

        # ───── Broadcast an andere Clients ─────
        with clients_lock:
            for client in list(clients.keys()):
                if time.time() - clients[client] > INACTIVITY_TIMEOUT:
                    log_event("INFO", "client_timeout", client=str(client))
                    del clients[client]
                    continue
                if client != from_address:
                    server.sendto(message, client)

# ────────────── Web Monitor ──────────────
app = Flask(__name__)

@app.route("/")
def index():
    with clients_lock:
        return jsonify({
            "clients": {
                str(addr): {
                    "packets": stats["packets"],
                    "lost": stats["lost"],
                    "jitter": round(stats["jitter"], 6),
                    "fec_needed": stats["fec_needed"],
                    "fec_failed": stats["fec_failed"]
                }
                for addr, stats in client_stats.items()
            }
        })

def run_web():
    app.run(host="0.0.0.0", port=8080)

# ────────────── Threads starten ──────────────
t1 = threading.Thread(target=receive, daemon=True)
t2 = threading.Thread(target=broadcast, daemon=True)
t3 = threading.Thread(target=run_web, daemon=True)

t1.start()
t2.start()
t3.start()

while True:
    time.sleep(1)