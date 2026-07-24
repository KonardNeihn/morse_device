import socket
import threading
import queue
import struct
import select
from datetime import datetime
from enum import Enum
import sys


HOST_NAME = socket.gethostname()
TCP_IP = socket.gethostbyname(HOST_NAME)
TCP_PORT = 5100
BUFFER_SIZE = 5

# ANSI-Farbcodes
COLORS = {
    "INFO": "\033[0m",      # Weiß
    "GOOD INFO": "\033[32m", # Grün
    "WARNING": "\033[33m",   # Gelb
    "ERROR": "\033[31m",     # Rot
    "RESET": "\033[0m",      # Reset
}

class LogLevel(Enum):
    INFO = "INFO"
    GOOD_INFO = "GOOD INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"

# Definiere globale Variablen für die Log-Levels
INFO = LogLevel.INFO
GOOD_INFO = LogLevel.GOOD_INFO
WARNING = LogLevel.WARNING
ERROR = LogLevel.ERROR

running = True

clients = []
clients_lock = threading.Lock()

# ==============================
# Main Server
# ==============================

def main():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(("0.0.0.0", TCP_PORT))
    server.listen()

    log(f"Server startet at {TCP_IP}:{TCP_PORT} name: {HOST_NAME}", GOOD_INFO)

    while True:
        try:
            client_socket, client_address = server.accept()
            client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 20)      # nach 20 s Inaktivität
            client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 10)     # alle 10 s erneut
            client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)        # 3 Versuche
            client_socket.setblocking(True)
        
            client = Clienthandler(client_socket, client_address)

            with clients_lock:
                clients.append(client)

        except Exception as e:
            log(f"Error: ({type(e).__name__}): {e}", ERROR)


# ==============================
# Client Handler
# ==============================

class Clienthandler:
    def __init__ (self, client_socket, client_address):
        self.client_socket = client_socket
        self.client_address = client_address

        self.out_queue = queue.Queue()
        self.running = True

        self.receive_thread = threading.Thread(target=self.receive_loop)
        self.receive_thread.start()

        self.send_thread = threading.Thread(target=self.send_loop)
        self.send_thread.start()

    def receive_loop(self):
        log(f"New client: {self.client_address}", GOOD_INFO)
        while self.running:
            try:
                ready, _, _ = select.select([self.client_socket], [], [], 1.0)

                if not ready:
                    continue

                # Zuerst status und length lesen (3 Bytes)
                header = self.client_socket.recv(3)
                if not header or len(header) < 3:
                    log(f"Unvollständiger Header von {self.client_address}", ERROR)
                    break

                status = header[0]
                length = (header[1] << 8) | header[2]

                signal_data = self.client_socket.recv(length)
                if not signal_data or len(signal_data) < length:
                    log(f"Unvollständige Payload von {self.client_address}. Erwartet: {length}, erhalten: {len(signal_data)}", ERROR)
                    break

                # Paket zusammenbauen (status, length, signal_data)
                packet = {"status": status, "length": length, "signal": signal_data}

                log(f"Package received: {self.client_address} {format_packet(packet)}", INFO)


                # keep alive zurück senden
                if status == 0:
                    response = {
                        "status": 0,
                        "length": 0,
                        "signal": b""
                    }

                    self.send(response)

                # Paket an alle anderen Clients weiterleiten
                broadcast(packet, self)

            except Exception as e:
                log(f"Error while receiving: ({type(e).__name__}): {e} with: {self.client_address}", ERROR)

        log(f"Client disconnected: {self.client_address}", WARNING)
        self.client_socket.close()
        self.running = False
        self.out_queue.put(None)
        with clients_lock:
            if self in clients:
                clients.remove(self)
    
    def send(self, packet):
        # packet ist ein Dictionary: {"status": uint8, "length": uint8, "signal": bytes}
        status = packet["status"]
        length = packet["length"]
        signal_data = packet["signal"]

        # Paket als Bytes serialisieren
        packet = struct.pack("!BH", status, length) + signal_data
        self.out_queue.put(packet)

    def send_loop(self):
        while self.running:
            try:
                packet = self.out_queue.get()
                if packet is None:
                    break
                self.client_socket.sendall(packet)
                log(f"Package sent to: {self.client_address}", INFO)
            except Exception as e:
                log(f"Error while sending: ({type(e).__name__}): {e} with: {self.client_address}", ERROR)


# ==============================
# Hilfsfunktionen
# ==============================

def broadcast (message, sender):
    with clients_lock:
        for client in clients:
            if client != sender:
                client.send(message)

def log(message, level=INFO):
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    color = COLORS.get(level.value, "")
    reset = COLORS["RESET"]
    print(f"{timestamp} {color}[{level.value}]{reset} {message}")

def format_packet(packet):
    status = packet["status"]
    length = packet["length"]
    signal = "".join("1" if byte else "0" for byte in packet["signal"])
    return f"Status: {status}, Length: {length}, Signal: {signal}"

# ==============================
# Start
# ==============================

if __name__ == "__main__":
    main()