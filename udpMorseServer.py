import socket
import threading
import queue
import time
import struct

INACTIVITY_TIMEOUT = 3600   # 1 Stunde
RECEIVE_PORT = 6969
BIND_IP = "192.168.178.103"

# ─────────────────────────────────────────────
# ESP32 Paket:
# struct UdpPacket {
#   uint8_t session;
#   MorseEvent current_event;
#   MorseEvent recent_event;
# };
#
# struct MorseEvent {
#   uint8_t  seq;
#   uint16_t duration_ms;
#   bool     state;
# };
#
# => 9 Bytes total, little endian, packed!
# ─────────────────────────────────────────────

PACKET_STRUCT = struct.Struct("<B B H B B H B")  # total 9 bytes

messages = queue.Queue()       # thread-safe
clients = {}                   # { address : last_seen }
clients_lock = threading.Lock()

print("Server IP:", socket.gethostbyname(socket.gethostname()))

server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server.bind((BIND_IP, RECEIVE_PORT))


def receive():
    while True:
        try:
            message, address = server.recvfrom(64)  # größer als nötig, sicher
            if len(message) != PACKET_STRUCT.size:
                print(f"Ignoring invalid packet size {len(message)} from {address}")
                continue

            messages.put((message, address))

            with clients_lock:
                clients[address] = time.time()

        except Exception as e:
            print(f"Error in receive(): {e}")


def broadcast():
    while True:
        try:
            message, from_address = messages.get()

            (
                session,
                cur_seq, cur_duration, cur_state,
                rec_seq, rec_duration, rec_state
            ) = PACKET_STRUCT.unpack(message)

            # Debug (optional)
            print(
                f"RX {from_address} | "
                f"S={session} | "
                f"CUR(seq={cur_seq}, dur={cur_duration}, state={cur_state}) | "
                f"REC(seq={rec_seq}, dur={rec_duration}, state={rec_state})"
            )

            to_remove = []

            with clients_lock:
                for client, last_seen in clients.items():
                    if time.time() - last_seen > INACTIVITY_TIMEOUT:
                        to_remove.append(client)
                        continue

                    if client != from_address:
                        server.sendto(message, client)

                for client in to_remove:
                    del clients[client]

        except Exception as e:
            print(f"Error in broadcast(): {e}")


t1 = threading.Thread(target=receive, daemon=True)
t2 = threading.Thread(target=broadcast, daemon=True)

t1.start()
t2.start()

print("UDP Morse relay server running...")

# Hauptthread am Leben halten
while True:
    time.sleep(1)
