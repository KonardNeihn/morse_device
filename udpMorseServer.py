import socket
import threading
import queue
import time
import struct
import json
import signal
from flask import Flask, jsonify, render_template_string, request

INACTIVITY_TIMEOUT = 3600
RECEIVE_PORT = 6969

PACKET_STRUCT = struct.Struct("<B B H B B H B")

messages = queue.Queue()
clients = {}
last_seq = {}
client_stats = {}

clients_lock = threading.Lock()
shutdown_event = threading.Event()

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
# UDP Setup (Auto IP Bind)
# ─────────────────────────────

server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server.bind(("0.0.0.0", RECEIVE_PORT))
server.settimeout(1.0)

print(f"UDP Server running on *:{RECEIVE_PORT}")
log_event("INFO", "server_start", port=RECEIVE_PORT)

# ─────────────────────────────
# RECEIVE THREAD
# ─────────────────────────────

def receive():
    while not shutdown_event.is_set():
        try:
            message, address = server.recvfrom(64)
            now = time.time()

            if len(message) != PACKET_STRUCT.size:
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
                        "history": []
                    }

                clients[address] = now

        except socket.timeout:
            continue
        except OSError:
            break

# ─────────────────────────────
# BROADCAST THREAD
# ─────────────────────────────

def broadcast():
    while not shutdown_event.is_set():
        try:
            message, from_address, arrival_time = messages.get(timeout=1)

            (
                session,
                cur_seq, cur_duration, cur_state,
                rec_seq, rec_duration, rec_state
            ) = PACKET_STRUCT.unpack(message)

            with clients_lock:
                stats = client_stats[from_address]
                stats["packets"] += 1

                # Packet loss
                if from_address in last_seq:
                    expected = (last_seq[from_address] + 1) & 0xFF
                    if cur_seq != expected:
                        lost = (cur_seq - expected) & 0xFF
                        stats["lost"] += lost

                last_seq[from_address] = cur_seq

                # Jitter
                if stats["last_arrival"] is not None:
                    delta = arrival_time - stats["last_arrival"]
                    stats["jitter"] = 0.9 * stats["jitter"] + 0.1 * abs(delta)

                stats["last_arrival"] = arrival_time

                stats["history"].append({
                    "time": arrival_time,
                    "jitter": stats["jitter"],
                    "lost": stats["lost"],
                    "packets": stats["packets"]
                })

                # History begrenzen
                if len(stats["history"]) > 100:
                    stats["history"].pop(0)

                # Broadcast
                for client in list(clients.keys()):
                    if time.time() - clients[client] > INACTIVITY_TIMEOUT:
                        del clients[client]
                        continue

                    if session == 0:
                        if client == from_address:
                            server.sendto(message, client)
                        continue

                    if client != from_address:
                        server.sendto(message, client)

        except queue.Empty:
            continue

# ─────────────────────────────
# WEB INTERFACE
# ─────────────────────────────

app = Flask(__name__)

HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
<title>Morse Server Monitor</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
</head>
<body>
<h2>Morse Server Live Monitor</h2>
<canvas id="chart" width="800" height="400"></canvas>

<script>
let ctx = document.getElementById('chart').getContext('2d');

let chart = new Chart(ctx, {
    type: 'line',
    data: {
        labels: [],
        datasets: [{
            label: 'Jitter',
            data: [],
            borderColor: 'blue',
            fill: false
        }]
    }
});

async function update() {
    let response = await fetch('/stats');
    let data = await response.json();

    let labels = [];
    let jitterData = [];

    for (let client in data.clients) {
        let history = data.clients[client].history;
        labels = history.map(x => new Date(x.time * 1000).toLocaleTimeString());
        jitterData = history.map(x => x.jitter);
    }

    chart.data.labels = labels;
    chart.data.datasets[0].data = jitterData;
    chart.update();
}

setInterval(update, 1000);
</script>
</body>
</html>
"""

@app.route("/")
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route("/stats")
def stats():
    with clients_lock:
        return jsonify({
            "clients": {
                str(addr): {
                    "packets": stats["packets"],
                    "lost": stats["lost"],
                    "jitter": stats["jitter"],
                    "history": stats["history"]
                }
                for addr, stats in client_stats.items()
            }
        })

# ─────────────────────────────
# SHUTDOWN HANDLER
# ─────────────────────────────

def shutdown_handler(sig, frame):
    print("Shutting down cleanly...")
    log_event("INFO", "server_shutdown")
    shutdown_event.set()
    server.close()

signal.signal(signal.SIGINT, shutdown_handler)
signal.signal(signal.SIGTERM, shutdown_handler)

# ─────────────────────────────
# START THREADS
# ─────────────────────────────

t1 = threading.Thread(target=receive)
t2 = threading.Thread(target=broadcast)
t3 = threading.Thread(target=lambda: app.run(host="0.0.0.0", port=8080, use_reloader=False))

t1.start()
t2.start()
t3.start()

t1.join()
t2.join()

print("Server terminated cleanly.")