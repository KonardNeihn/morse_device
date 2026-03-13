import asyncio
import struct

FRAMES_PER_PACKET = 8
SERVER_PORT = 6969

# Packet format: status (1 byte) + signal (FRAMES_PER_PACKET bytes)
PACKET_STRUCT = struct.Struct(f"!B{FRAMES_PER_PACKET}B")  # network byte order

# Queues für Broadcast
rx_queue = asyncio.Queue()
tx_queue = asyncio.Queue()


async def handle_client(reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
    addr = writer.get_extra_info('peername')
    print(f"Client connected: {addr}")

    try:
        while True:
            # ======================
            # 1) RX
            # ======================
            data = await reader.readexactly(PACKET_STRUCT.size)
            pkt = PACKET_STRUCT.unpack(data)
            status, signal = pkt[0], pkt[1:]

            # Server Check Paket
            if status == 1:
                # direkt zurücksenden
                await tx_queue.put((status, signal))
                continue

            # Ping Paket
            if status == 2:
                print(f"Ping received from {addr}")
                continue

            # Normales Signal
            print(f"Received signal from {addr}: {[hex(b) for b in signal]}")
            await rx_queue.put((status, signal))

            # ======================
            # 2) TX: Broadcast alle Pakete aus tx_queue
            # ======================
            while not tx_queue.empty():
                out_status, out_signal = await tx_queue.get()
                out_bytes = PACKET_STRUCT.pack(out_status, *out_signal)
                try:
                    writer.write(out_bytes)
                    await writer.drain()
                except Exception as e:
                    print(f"Failed to send packet to {addr}: {e}")
                    return

    except asyncio.IncompleteReadError:
        print(f"Client disconnected: {addr}")
    except Exception as e:
        print(f"Error with client {addr}: {e}")
    finally:
        writer.close()
        await writer.wait_closed()


async def server_loop():
    server = await asyncio.start_server(handle_client, "0.0.0.0", SERVER_PORT)
    print(f"Server listening on port {SERVER_PORT}")
    async with server:
        await server.serve_forever()


async def processing_loop():
    while True:
        status, signal = await rx_queue.get()
        # Hier kann zusätzliche Logik für Signale eingefügt werden,
        # z.B. Weiterverarbeitung, Logging, Broadcasting usw.
        # Momentan senden wir einfach alles an tx_queue (Broadcast)
        await tx_queue.put((status, signal))


async def main():
    # Starte Tasks
    await asyncio.gather(server_loop(), processing_loop())


if __name__ == "__main__":
    asyncio.run(main())