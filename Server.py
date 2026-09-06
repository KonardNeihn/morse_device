import socket
import threading
import queue
import struct
import select
import time
from datetime import datetime
from enum import Enum
from collections import OrderedDict, deque

TCP_PORT = 6969

# Status-Codes (müssen zum esp32-client passen)
STATUS_KEEPALIVE = 0   # Verbindung am Leben halten
STATUS_MESSAGE   = 1   # normale Morse-Nachricht (Broadcast)
STATUS_ACK       = 2   # Bestätigung; Bedeutung je Richtung:
                       #   Client -> Server : Zustell-ACK (msg_id = server_msg_id)
                       #   Server -> Client : Annahme-ACK (msg_id = client_msg_id)
STATUS_CHECK     = 3   # Server-Check: zurücksenden statt broadcasten
STATUS_REGISTER  = 4   # Registrierung: Payload = 6-Byte-MAC

# Paket-Header: status (1 B) | msg_id (8 B) | size (2 B) | payload
HEADER = struct.Struct("!BQH")

TTL_SECONDS = 7 * 24 * 3600   # so lange offline -> MAC + Puffer vergessen
RECENT_MAX = 32               # gemerkte client_msg_id -> server_msg_id je Client

# ANSI-Farbcodes
COLORS = {
    "INFO": "\033[0m",
    "GOOD INFO": "\033[32m",
    "WARNING": "\033[33m",
    "ERROR": "\033[31m",
    "RESET": "\033[0m",
}

class LogLevel(Enum):
    INFO = "INFO"
    GOOD_INFO = "GOOD INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"

INFO = LogLevel.INFO
GOOD_INFO = LogLevel.GOOD_INFO
WARNING = LogLevel.WARNING
ERROR = LogLevel.ERROR

# Aktive Verbindungen (jede mit eigenem Sende-/Empfangs-Thread).
clients = []
clients_lock = threading.Lock()

# Bekannte Geräte, Schlüssel = MAC (6 Bytes):
#   "handler"  : Clienthandler | None (None = offline)
#   "last_seen": letzter Kontakt (Unix-Zeit)
#   "buffer"   : deque[(server_msg_id, payload)]   -> wartende Nachrichten
#   "recent"   : OrderedDict[client_msg_id] = server_msg_id (Retry-Erkennung)
registry = {}
registry_lock = threading.Lock()
next_server_msg_id = 0


def new_server_msg_id():
    """Vergibt die nächste (globale, monotone) server_msg_id."""
    global next_server_msg_id
    mid = next_server_msg_id
    next_server_msg_id += 1
    return mid


def log(message, level=INFO):
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    color = COLORS.get(level.value, "")
    reset = COLORS["RESET"]
    print(f"{timestamp} {color}[{level.value:<9}]{reset} {message}")


def fmt_mac(mac):
    """Formatiert 6 MAC-Bytes als 'aa:bb:cc:dd:ee:ff'."""
    return ":".join(f"{b:02x}" for b in mac)


def log_event(direction, event, mac=None, msg_id=None, length=None, note=""):
    """Eine fest formatierte Tabellenzeile für einen Protokoll-Event."""
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    mac_s = fmt_mac(mac) if mac is not None else "-"
    mid_s = str(msg_id) if msg_id is not None else "-"
    len_s = str(length) if length is not None else "-"
    print(f"{ts}  {direction:<4} {event:<14} {mac_s:<17} {mid_s:<20} {len_s:<5} {note}")


def log_table_header():
    """Druckt die Spaltenüberschrift der Event-Tabelle (einmal beim Start)."""
    print(f"{'TIME':<12}  {'DIR':<4} {'EVENT':<14} {'MAC':<17} {'MSG_ID':<20} {'LEN':<5} NOTE")


def reaper_loop():
    """Löscht periodisch MACs, die lange offline waren (inkl. Puffer)."""
    while True:
        time.sleep(60)
        now = time.time()
        with registry_lock:
            for mac in list(registry.keys()):
                entry = registry[mac]
                if entry["handler"] is None and now - entry["last_seen"] > TTL_SECONDS:
                    del registry[mac]
                    log(f"Forgot offline MAC:    {fmt_mac(mac)}", WARNING)


def main():
    running = True

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", TCP_PORT))
    server.listen()
    server.settimeout(1.0)      # wichtig für kontrolliertes Schließen

    reaper = threading.Thread(target=reaper_loop, daemon=True)
    reaper.start()

    log("Server startet...", GOOD_INFO)
    log_table_header()

    try:
        while running:
            try:
                client_socket, client_address = server.accept()
                client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 10)
                client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 5)
                client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)
                client_socket.settimeout(1.0)

            except socket.timeout:
                continue

            except Exception as e:
                log(f"Error while connecting: ({type(e).__name__}): {e}", ERROR)
                break

            client = Clienthandler(client_socket, client_address)
            with clients_lock:
                clients.append(client)

    except KeyboardInterrupt:
        log("Server wird beendet...", WARNING)

    except Exception as e:
        log(f"Error in main server: ({type(e).__name__}): {e}", ERROR)

    finally:
        running = False
        server.close()
        # Kopie, um Deadlocks beim Stoppen zu vermeiden
        with clients_lock:
            current_clients = clients.copy()

        for client in current_clients:
            client.stop()


class Clienthandler:
    def __init__(self, client_socket, client_address):
        self.client_socket = client_socket
        self.client_address = client_address
        self.mac = None          # nach Registrierung gesetzt
        self.out_queue = queue.Queue()
        self.running = True

        self.receive_thread = threading.Thread(target=self.receive_loop, daemon=True)
        self.receive_thread.start()
        self.send_thread = threading.Thread(target=self.send_loop, daemon=True)
        self.send_thread.start()

    def receive_loop(self):
        log_event("----", "CONNECT", note=f"addr={self.client_address}")
        reason = "stopped"
        while self.running:
            try:
                ready, _, _ = select.select([self.client_socket], [], [], 1.0)
                if not ready:
                    continue

                header, reason = self.recv_exact(HEADER.size)
                if header is None:
                    break

                status, msg_id, length = HEADER.unpack(header)

                payload, reason = self.recv_exact(length)
                if payload is None:
                    break

                self.handle_packet(status, msg_id, payload)

            except Exception as e:
                reason = f"error:{type(e).__name__}"
                break

        self._on_disconnect(reason)

    def handle_packet(self, status, msg_id, payload):
        if status == STATUS_KEEPALIVE:
            log_event("RECV", "KEEPALIVE", mac=self.mac, note=f"addr={self.client_address}")
            self.send(HEADER.pack(STATUS_KEEPALIVE, 0, 0))
            return
        if status == STATUS_REGISTER:
            self._handle_register(payload)
            return
        if status == STATUS_ACK:
            self._handle_ack(msg_id)
            return
        if status == STATUS_CHECK:
            self._handle_check(msg_id, payload)
            return
        if status == STATUS_MESSAGE:
            self._handle_message(msg_id, payload)
            return
        log_event("----", "ERROR", mac=self.mac, note=f"unknown status={status}")

    # --------------------------------------------------------------
    # Registrierung
    # --------------------------------------------------------------
    def _handle_register(self, payload):
        if len(payload) != 6:
            log_event("----", "ERROR", mac=self.mac, note=f"register len={len(payload)}")
            return

        mac = payload
        old_handler = None
        buffered = []

        with registry_lock:
            entry = registry.get(mac)
            if entry is None:
                entry = {"handler": self, "last_seen": time.time(),
                         "buffer": deque(), "recent": OrderedDict()}
                registry[mac] = entry
            else:
                old_handler = entry["handler"]
                entry["handler"] = self
                entry["last_seen"] = time.time()
                buffered = list(entry["buffer"])

            self.mac = mac

        # Alte Verbindung derselben MAC stoppen (außerhalb des Locks).
        if old_handler is not None and old_handler is not self:
            log_event("----", "DUPLICATE", mac=mac, note="stopping old connection")
            old_handler.mac = None
            old_handler.stop()

        log_event("RECV", "REGISTER", mac=mac, note=f"addr={self.client_address}")

        # Gepufferte Nachrichten an den gerade verbundenen Client ausliefern.
        for server_msg_id, pkt in buffered:
            self.send(HEADER.pack(STATUS_MESSAGE, server_msg_id, len(pkt)) + pkt)
            log_event("SEND", "DELIVERY", mac=mac, msg_id=server_msg_id, length=len(pkt), note="buffered")
        if buffered:
            log(f"Flushed {len(buffered)} buffered to {fmt_mac(mac)}", INFO)

    # --------------------------------------------------------------
    # ACK (Zustell-ACK eines Empfängers)
    # --------------------------------------------------------------
    def _handle_ack(self, msg_id):
        if self.mac is None:
            log_event("----", "ERROR", mac=self.mac, note="ACK before register")
            return
        with registry_lock:
            entry = registry.get(self.mac)
            if entry is not None:
                entry["last_seen"] = time.time()
                # Nachricht mit dieser server_msg_id als zugestellt markieren.
                entry["buffer"] = deque(
                    item for item in entry["buffer"] if item[0] != msg_id
                )
        log_event("RECV", "DELIVERY_ACK", mac=self.mac, msg_id=msg_id)

    # --------------------------------------------------------------
    # Server-Check: zurücksenden statt broadcasten
    # --------------------------------------------------------------
    def _handle_check(self, client_msg_id, payload):
        if self.mac is None:
            log_event("----", "ERROR", mac=self.mac, note="check before register")
            return

        with registry_lock:
            entry = registry.get(self.mac)
            if entry is None:
                return
            entry["last_seen"] = time.time()

            if client_msg_id in entry["recent"]:
                # Retry: bereits bearbeitet -> Echo + Annahme erneut senden.
                server_msg_id = entry["recent"][client_msg_id]
                log_event("RECV", "CHECK", mac=self.mac, msg_id=client_msg_id, length=len(payload), note="retry")
                self.send(HEADER.pack(STATUS_CHECK, server_msg_id, len(payload)) + payload)
                self.send(HEADER.pack(STATUS_ACK, client_msg_id, 0))
                return

            server_msg_id = new_server_msg_id()
            entry["recent"][client_msg_id] = server_msg_id
            self._trim_recent(entry)

        log_event("RECV", "CHECK", mac=self.mac, msg_id=client_msg_id, length=len(payload))
        self.send(HEADER.pack(STATUS_CHECK, server_msg_id, len(payload)) + payload)
        self.send(HEADER.pack(STATUS_ACK, client_msg_id, 0))
        log_event("SEND", "CHECK_ECHO", mac=self.mac, msg_id=server_msg_id, length=len(payload))
        log_event("SEND", "ACCEPT_ACK", mac=self.mac, msg_id=client_msg_id)

    # --------------------------------------------------------------
    # Normale Nachricht: broadcasten + für Offline-Clients puffern
    # --------------------------------------------------------------
    def _handle_message(self, client_msg_id, payload):
        if self.mac is None:
            log_event("----", "ERROR", mac=self.mac, note="message before register")
            return

        with registry_lock:
            entry = registry.get(self.mac)
            if entry is None:
                return
            entry["last_seen"] = time.time()

            if client_msg_id in entry["recent"]:
                # Retry: bereits gebroadcastet -> nur Annahme erneut senden.
                log_event("RECV", "MESSAGE", mac=self.mac, msg_id=client_msg_id, length=len(payload), note="retry")
                self.send(HEADER.pack(STATUS_ACK, client_msg_id, 0))
                return

            server_msg_id = new_server_msg_id()
            entry["recent"][client_msg_id] = server_msg_id
            self._trim_recent(entry)

            # Für ALLE anderen Clients puffern (auch online, falls "fake-online").
            # Online-Clients bekommen die Nachricht zusätzlich sofort per broadcast;
            # der Buffer wird erst beim Delivery-ACK entfernt.
            for other_mac, other in registry.items():
                if other_mac == self.mac:
                    continue
                other["buffer"].append((server_msg_id, payload))

        log_event("RECV", "MESSAGE", mac=self.mac, msg_id=client_msg_id, length=len(payload))
        packet = HEADER.pack(STATUS_MESSAGE, server_msg_id, len(payload)) + payload
        broadcast(packet, self)
        self.send(HEADER.pack(STATUS_ACK, client_msg_id, 0))
        log_event("SEND", "BROADCAST", mac=self.mac, msg_id=server_msg_id, length=len(payload))
        log_event("SEND", "ACCEPT_ACK", mac=self.mac, msg_id=client_msg_id)

    @staticmethod
    def _trim_recent(entry):
        while len(entry["recent"]) > RECENT_MAX:
            entry["recent"].popitem(last=False)

    def _on_disconnect(self, reason="unknown"):
        log_event("----", "DISCONNECT", mac=self.mac, note=f"reason={reason} addr={self.client_address}")
        try:
            self.client_socket.close()
        except OSError:
            pass
        self.running = False
        self.out_queue.put(None)

        with clients_lock:
            if self in clients:
                clients.remove(self)

        with registry_lock:
            entry = registry.get(self.mac)
            if entry is not None and entry["handler"] is self:
                entry["handler"] = None
                entry["last_seen"] = time.time()

    def recv_exact(self, size):
        """Liest exakt `size` Bytes. Gibt (bytes, None) bei Erfolg zurück,
        sonst (None, grund). `grund` beschreibt den Abbruchgrund:
        'closed' (FIN), 'reset' (RST), 'timeout' oder 'stopped'."""
        data = b""
        while len(data) < size and self.running:
            try:
                chunk = self.client_socket.recv(size - len(data))
                if not chunk:
                    return None, "closed"
                data += chunk
            except socket.timeout:
                # Teil-Paket erhalten, aber >1 s keine weiteren Bytes -> hängt.
                return None, "timeout"
            except ConnectionResetError:
                return None, "reset"
            except OSError as e:
                return None, f"error:{type(e).__name__}"
        if not self.running:
            return None, "stopped"
        return data, None

    def send(self, packet_bytes):
        self.out_queue.put(packet_bytes)

    def send_loop(self):
        while self.running:
            packet = self.out_queue.get()
            if packet is None:
                break
            try:
                self.client_socket.sendall(packet)
            except Exception as e:
                log_event("----", "ERROR", mac=self.mac, note=f"send: {type(e).__name__}")

    def stop(self):
        self.running = False
        try:
            self.client_socket.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            self.client_socket.close()
        except OSError:
            pass
        self.out_queue.put(None)
        self.receive_thread.join(timeout=2)
        self.send_thread.join(timeout=2)


def broadcast(packet_bytes, sender):
    with clients_lock:
        for client in clients:
            if client is not sender:
                client.send(packet_bytes)


if __name__ == "__main__":
    main()
