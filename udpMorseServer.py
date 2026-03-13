#!/usr/bin/env python3
import asyncio
import struct
import time
from collections import deque

# ==============================
# Server Konfiguration
# ==============================
HOST = "0.0.0.0"
PORT = 6969

SIGNAL_BYTES = 8          # <-- HIER kann man ändern, wie viele Bytes pro Signal
PACKET_SIZE = 1 + SIGNAL_BYTES  # 1 Byte Status + SignalBytes

PING_INTERVAL = 10.0     # Sekunden
MAX_QUEUE_PACKETS = 256
MAX_QUEUE_BYTES = 4096

# Dynamische Struct für Packets
PACKET_STRUCT = struct.Struct(f"B{SIGNAL_BYTES}B")

# ==============================
# Client Management
# ==============================
class Client:
    def __init__(self, reader, writer):
        self.reader = reader
        self.writer = writer
        self.addr = writer.get_extra_info('peername')
        self.tx_queue = deque()
        self.rx_buffer = bytearray()
        self.last_ping = time.time()
        self.connected = True

clients = set()

# ==============================
# Hilfsfunktionen
# ==============================
def parse_packets(buffer):
    packets = []
    while len(buffer) >= PACKET_SIZE:
        pkt_bytes = buffer[:PACKET_SIZE]
        buffer = buffer[PACKET_SIZE:]
        pkt = PACKET_STRUCT.unpack(pkt_bytes)
        packets.append(pkt)
    return packets, buffer

async def broadcast(packet, exclude=None):
    for client in clients:
        if client is exclude or not client.connected:
            continue
        if len(client.tx_queue) >= MAX_QUEUE_PACKETS:
            client.tx_queue.popleft()  # drop_oldest
        client.tx_queue.append(packet)

async def handle_client(client: Client):
    print(f"[+] client connected: {client.addr}")
    try:
        while client.connected:
            data = await client.reader.read(1024)
            if not data:
                break
            client.rx_buffer.extend(data)
            packets, client.rx_buffer = parse_packets(client.rx_buffer)
            for pkt in packets:
                status = pkt[0]
                # Ping Paket
                if status == 2:
                    client.last_ping = time.time()
                    continue
                # Broadcast an alle anderen Clients
                await broadcast(pkt, exclude=client)
    except Exception as e:
        print(f"[-] client error {client.addr}: {e}")
    finally:
        client.connected = False
        clients.discard(client)
        client.writer.close()
        await client.writer.wait_closed()
        print(f"[-] client disconnected: {client.addr}")

async def tx_loop(client: Client):
    try:
        while client.connected:
            if client.tx_queue:
                pkt = client.tx_queue.popleft()
                try:
                    client.writer.write(PACKET_STRUCT.pack(*pkt))
                    await client.writer.drain()
                except Exception as e:
                    print(f"[-] TX error {client.addr}: {e}")
                    client.connected = False
                    break
            else:
                await asyncio.sleep(0.001)
    except Exception as e:
        print(f"[-] TX loop error {client.addr}: {e}")
    finally:
        client.connected = False

async def ping_check():
    while True:
        now = time.time()
        for client in list(clients):
            if not client.connected:
                continue
            if now - client.last_ping > PING_INTERVAL * 3:
                print(f"[-] client timeout: {client.addr}")
                client.connected = False
        await asyncio.sleep(1)

async def accept_clients(reader, writer):
    client = Client(reader, writer)
    clients.add(client)
    await asyncio.gather(handle_client(client), tx_loop(client))

# ==============================
# Main
# ==============================
async def main():
    server = await asyncio.start_server(accept_clients, HOST, PORT)
    print(f"Listening on {HOST}:{PORT}")
    asyncio.create_task(ping_check())
    async with server:
        await server.serve_forever()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Shutting down...")
        for client in clients:
            client.connected = False