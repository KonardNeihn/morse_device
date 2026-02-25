import socket
import threading
import queue
import time
import struct
import json
from flask import Flask, jsonify

# ────────────── CONFIG ──────────────
INACTIVITY_TIMEOUT = 3600
RECEIVE_PORT = 6969
SEND_PORT = 6969
FRAMES_PER_BLOCK = 4

# Packet: type (DATA=0,FEC=1), block_id, frame_id, timestamp (ms), signal
PACKET_STRUCT = struct.Struct("<B B B I B")

# Queues und Locks
messages = queue.Queue()
clients = {}
blocks = {}
client_stats = {}
clients_lock = threading.Lock()

# ────────────── Logging ──────────────
def log_event(level, event, **kwargs):
    entry = {
        "timestamp": time.time(),
        "level": level,
        "event": event,
        **kwargs
    }
    print(entry)
    with open("morse_server.jsonl", "a") as f:
        f.write(json.dumps(entry) + "\n")

# ────────────── Socket Setup ──────────────
server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server.bind(("0.0.0.0", RECEIVE_PORT))
server.setblocking(True)

log_event("INFO", "server_start", port=RECEIVE_PORT)

# ────────────── Receiver Thread ──────────────
def receive():
    while True:
        try:
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

        except Exception as e:
            log_event("ERROR", "recv_exception", error=str(e))

# ────────────── Broadcast & FEC Recovery ──────────────
def broadcast():
    while True:
        try:
            message, from_address, arrival_time = messages.get()
            pkt_type, block_id, frame_id, timestamp, signal = PACKET_STRUCT.unpack(message)

            # ───── Statistik Upstream ─────
            with clients_lock:
                stats = client_stats[from_address]
                stats["packets"] += 1

                if stats["last_arrival"] is not None:
                    delta = arrival_time - stats["last_arrival"]
                    stats["jitter"] = 0.9 * stats["jitter"] + 0.1 * abs(delta)
                stats["last_arrival"] = arrival_time

            # ───── Block Handling ─────
            if block_id not in blocks:
                blocks[block_id] = {
                    "frames": [None]*FRAMES_PER_BLOCK,
                    "fec": None,
                    "mask": 0,
                }

            block = blocks[block_id]

            if pkt_type == 0:  # DATA
                block["frames"][frame_id] = signal
                block["mask"] |= (1 << frame_id)
            elif pkt_type == 1:  # FEC
                block["fec"] = signal

            # FEC Recovery: wenn genau 1 Frame fehlt
            missing_count = FRAMES_PER_BLOCK - bin(block["mask"]).count("1")
            if missing_count == 1 and block["fec"] is not None:
                missing_index = [i for i, v in enumerate(block["frames"]) if v is None][0]
                recovered = block["fec"]
                for i, val in enumerate(block["frames"]):
                    if i != missing_index:
                        recovered ^= val
                block["frames"][missing_index] = recovered
                block["mask"] |= (1 << missing_index)
                log_event("INFO", "fec_recovered", block=block_id, frame=missing_index)

            # ───── Block Complete → Broadcast ─────
            if block["mask"] == (1 << FRAMES_PER_BLOCK) - 1:
                with clients_lock:
                    for client in list(clients.keys()):
                        # Timeout-Check
                        if time.time() - clients[client] > INACTIVITY_TIMEOUT:
                            log_event("INFO", "client_timeout", client=str(client))
                            del clients[client]
                            continue
                        # Broadcast an alle außer Quelle
                        if client != from_address:
                            for i, val in enumerate(block["frames"]):
                                pkt = PACKET_STRUCT.pack(0, block_id, i, int(time.time()*1000)%0xFFFFFFFF, val)
                                server.sendto(pkt, client)

                del blocks[block_id]

        except Exception as e:
            log_event("ERROR", "broadcast_exception", error=str(e))

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
                    "jitter": round(stats["jitter"], 6)
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