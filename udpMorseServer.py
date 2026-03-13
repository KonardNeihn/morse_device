import socket
import selectors
import types
import time
import collections
import struct

HOST = "0.0.0.0"
PORT = 4242

PACKET_SIZE = 5
PING_INTERVAL = 10.0

# status byte meanings
STATUS_NORMAL = 0
STATUS_SERVER_CHECK = 1
STATUS_PING = 2

# Per-client TX queue limits
MAX_TX_QUEUE_PACKETS = 256       # how many packets buffered per client
MAX_TX_BUFFER_BYTES = 4096       # max total unsent bytes per client before drop/kick policy

# Policy when client TX queue is full:
# "drop_oldest" -> remove oldest packet and enqueue new one
# "drop_newest" -> reject new packet
# "disconnect"  -> disconnect slow client
TX_OVERFLOW_POLICY = "drop_oldest"


class Stats:
    def __init__(self):
        self.start_time = time.time()

        self.accepted = 0
        self.disconnected = 0

        self.rx_packets = 0
        self.rx_bytes = 0

        self.tx_packets_enqueued = 0
        self.tx_packets_sent = 0
        self.tx_bytes_sent = 0

        self.broadcasts = 0

        self.tx_queue_overflows = 0
        self.tx_drop_oldest = 0
        self.tx_drop_newest = 0
        self.tx_disconnect_slow = 0

        self.partial_sends = 0
        self.read_errors = 0
        self.write_errors = 0
        self.malformed = 0

    def snapshot(self):
        uptime = time.time() - self.start_time
        return {
            "uptime_s": round(uptime, 1),
            "accepted": self.accepted,
            "disconnected": self.disconnected,
            "rx_packets": self.rx_packets,
            "rx_bytes": self.rx_bytes,
            "tx_packets_enqueued": self.tx_packets_enqueued,
            "tx_packets_sent": self.tx_packets_sent,
            "tx_bytes_sent": self.tx_bytes_sent,
            "broadcasts": self.broadcasts,
            "tx_queue_overflows": self.tx_queue_overflows,
            "tx_drop_oldest": self.tx_drop_oldest,
            "tx_drop_newest": self.tx_drop_newest,
            "tx_disconnect_slow": self.tx_disconnect_slow,
            "partial_sends": self.partial_sends,
            "read_errors": self.read_errors,
            "write_errors": self.write_errors,
            "malformed": self.malformed,
        }


stats = Stats()
sel = selectors.DefaultSelector()
clients = {}  # sock -> ClientState


class ClientState:
    def __init__(self, sock, addr):
        self.sock = sock
        self.addr = addr

        self.rx_buffer = bytearray()

        # Queue of complete packets waiting to be sent (bytes objects)
        self.tx_queue = collections.deque()

        # Current packet being partially sent
        self.current_tx = None
        self.current_tx_offset = 0

        self.last_rx = time.time()
        self.last_tx = time.time()

        self.total_queued_bytes = 0

        self.id = f"{addr[0]}:{addr[1]}"

    def __repr__(self):
        return f"<ClientState {self.id} q={len(self.tx_queue)} bytes={self.total_queued_bytes} partial={self.current_tx_offset}>"

    def has_pending_tx(self):
        return self.current_tx is not None or len(self.tx_queue) > 0


def set_socket_options(sock):
    sock.setblocking(False)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    # Optional: keepalive
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    except OSError:
        pass


def update_interest(client: ClientState):
    events = selectors.EVENT_READ
    if client.has_pending_tx():
        events |= selectors.EVENT_WRITE

    try:
        sel.modify(client.sock, events, data=client)
    except KeyError:
        # not yet registered
        sel.register(client.sock, events, data=client)
    except Exception as e:
        print(f"[WARN] modify failed for {client.id}: {e}")
        disconnect_client(client)


def accept_connection(server_sock):
    try:
        conn, addr = server_sock.accept()
    except BlockingIOError:
        return
    except Exception as e:
        print(f"[ERR] accept failed: {e}")
        return

    set_socket_options(conn)
    client = ClientState(conn, addr)
    clients[conn] = client
    stats.accepted += 1

    sel.register(conn, selectors.EVENT_READ, data=client)
    print(f"[+] client connected: {client.id}")


def disconnect_client(client: ClientState):
    sock = client.sock
    if sock not in clients:
        return

    print(f"[-] client disconnected: {client.id} (queued={len(client.tx_queue)} partial={client.current_tx_offset})")

    stats.disconnected += 1

    try:
        sel.unregister(sock)
    except Exception:
        pass

    try:
        sock.close()
    except Exception:
        pass

    clients.pop(sock, None)


def enqueue_packet(client: ClientState, packet: bytes):
    """
    Queue one complete packet (exactly PACKET_SIZE bytes) for a client.
    Applies overflow policy if queue grows too large.
    """
    if len(packet) != PACKET_SIZE:
        stats.malformed += 1
        return False

    # Estimate queued packets incl. current partial packet
    queued_packets = len(client.tx_queue) + (1 if client.current_tx is not None else 0)
    projected_bytes = client.total_queued_bytes + len(packet)

    if queued_packets >= MAX_TX_QUEUE_PACKETS or projected_bytes > MAX_TX_BUFFER_BYTES:
        stats.tx_queue_overflows += 1

        if TX_OVERFLOW_POLICY == "drop_oldest":
            # Prefer dropping from tx_queue first
            if client.tx_queue:
                dropped = client.tx_queue.popleft()
                client.total_queued_bytes -= len(dropped)
                stats.tx_drop_oldest += 1
            elif client.current_tx is not None:
                # If only current partial exists, disconnect is safer than corrupting partial send
                stats.tx_disconnect_slow += 1
                print(f"[SLOW] {client.id}: overflow with partial packet active -> disconnect")
                disconnect_client(client)
                return False
            else:
                # weird edge case
                stats.tx_drop_oldest += 1

        elif TX_OVERFLOW_POLICY == "drop_newest":
            stats.tx_drop_newest += 1
            return False

        elif TX_OVERFLOW_POLICY == "disconnect":
            stats.tx_disconnect_slow += 1
            print(f"[SLOW] {client.id}: TX queue overflow -> disconnect")
            disconnect_client(client)
            return False

    client.tx_queue.append(packet)
    client.total_queued_bytes += len(packet)
    stats.tx_packets_enqueued += 1

    update_interest(client)
    return True


def broadcast_packet(sender: ClientState, packet: bytes):
    """
    Broadcast one packet to all OTHER clients.
    """
    stats.broadcasts += 1

    dead = []
    for sock, client in clients.items():
        if client is sender:
            continue
        ok = enqueue_packet(client, packet)
        if not ok and sock not in clients:
            dead.append(client)

    # dead already removed by disconnect_client if needed


def handle_read(client: ClientState):
    try:
        data = client.sock.recv(4096)
    except BlockingIOError:
        return
    except ConnectionResetError:
        stats.read_errors += 1
        disconnect_client(client)
        return
    except Exception as e:
        stats.read_errors += 1
        print(f"[ERR] recv {client.id}: {e}")
        disconnect_client(client)
        return

    if not data:
        disconnect_client(client)
        return

    client.last_rx = time.time()
    stats.rx_bytes += len(data)

    client.rx_buffer.extend(data)

    # Parse complete packets
    while len(client.rx_buffer) >= PACKET_SIZE:
        packet = bytes(client.rx_buffer[:PACKET_SIZE])
        del client.rx_buffer[:PACKET_SIZE]

        stats.rx_packets += 1

        # Optional: inspect packet
        # signal = struct.unpack(">I", packet[:4])[0]
        status = packet[4]

        # If you want server to react to status:
        # STATUS_PING from clients is simply rebroadcasted like any other packet.
        # STATUS_SERVER_CHECK can also be rebroadcasted.
        # For now: broadcast everything except back to sender.
        broadcast_packet(client, packet)


def handle_write(client: ClientState):
    # Load next packet into partial-send slot if needed
    if client.current_tx is None and client.tx_queue:
        client.current_tx = client.tx_queue.popleft()
        client.total_queued_bytes -= len(client.current_tx)
        client.current_tx_offset = 0

    if client.current_tx is None:
        update_interest(client)
        return

    try:
        view = memoryview(client.current_tx)[client.current_tx_offset:]
        sent = client.sock.send(view)
    except BlockingIOError:
        # socket not actually writable right now
        return
    except BrokenPipeError:
        stats.write_errors += 1
        disconnect_client(client)
        return
    except ConnectionResetError:
        stats.write_errors += 1
        disconnect_client(client)
        return
    except Exception as e:
        stats.write_errors += 1
        print(f"[ERR] send {client.id}: {e}")
        disconnect_client(client)
        return

    if sent == 0:
        # Treat as dead connection
        stats.write_errors += 1
        disconnect_client(client)
        return

    client.last_tx = time.time()
    stats.tx_bytes_sent += sent

    if sent < len(view):
        # partial send
        client.current_tx_offset += sent
        stats.partial_sends += 1
    else:
        # packet finished
        client.current_tx = None
        client.current_tx_offset = 0
        stats.tx_packets_sent += 1

    update_interest(client)


def send_ping_to_all():
    ping_packet = struct.pack(">IB", 0, STATUS_PING)

    for sock, client in list(clients.items()):
        enqueue_packet(client, ping_packet)


def print_stats():
    snap = stats.snapshot()
    print(
        "[STATS] "
        f"up={snap['uptime_s']}s "
        f"clients={len(clients)} "
        f"acc={snap['accepted']} disc={snap['disconnected']} "
        f"rx_pkt={snap['rx_packets']} rx_b={snap['rx_bytes']} "
        f"tx_enq={snap['tx_packets_enqueued']} tx_pkt={snap['tx_packets_sent']} tx_b={snap['tx_bytes_sent']} "
        f"bcast={snap['broadcasts']} "
        f"qovf={snap['tx_queue_overflows']} "
        f"drop_old={snap['tx_drop_oldest']} drop_new={snap['tx_drop_newest']} slow_disc={snap['tx_disconnect_slow']} "
        f"partial={snap['partial_sends']} "
        f"rerr={snap['read_errors']} werr={snap['write_errors']}"
    )

    # Optional per-client queue stats
    for client in clients.values():
        print(
            f"    {client.id} "
            f"rxbuf={len(client.rx_buffer)} "
            f"txq={len(client.tx_queue)} "
            f"txbytes={client.total_queued_bytes} "
            f"partial={'yes' if client.current_tx is not None else 'no'} "
            f"partial_off={client.current_tx_offset}"
        )


def main():
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    server_sock.bind((HOST, PORT))
    server_sock.listen()
    server_sock.setblocking(False)

    sel.register(server_sock, selectors.EVENT_READ, data=None)

    print(f"Listening on {HOST}:{PORT}")
    print(f"PACKET_SIZE={PACKET_SIZE}, ping every {PING_INTERVAL}s")
    print(f"TX overflow policy={TX_OVERFLOW_POLICY}, max_queue_packets={MAX_TX_QUEUE_PACKETS}, max_queue_bytes={MAX_TX_BUFFER_BYTES}")

    next_ping = time.time() + PING_INTERVAL
    next_stats = time.time() + 5.0

    try:
        while True:
            timeout = min(max(0.0, next_ping - time.time()), 1.0)
            events = sel.select(timeout)

            for key, mask in events:
                if key.data is None:
                    # server socket
                    accept_connection(key.fileobj)
                    continue

                client = key.data

                # Client may get disconnected in read, so re-check after read
                if mask & selectors.EVENT_READ:
                    handle_read(client)
                    if client.sock not in clients:
                        continue

                if mask & selectors.EVENT_WRITE:
                    handle_write(client)
                    if client.sock not in clients:
                        continue

            now = time.time()

            if now >= next_ping:
                send_ping_to_all()
                next_ping = now + PING_INTERVAL

            if now >= next_stats:
                print_stats()
                next_stats = now + 5.0

    except KeyboardInterrupt:
        print("\nShutting down...")

    finally:
        for client in list(clients.values()):
            disconnect_client(client)

        try:
            sel.unregister(server_sock)
        except Exception:
            pass

        server_sock.close()


if __name__ == "__main__":
    main()