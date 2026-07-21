import socket
import threading
import queue
import struct

HOST_NAME = socket.gethostname()
TCP_IP = socket.gethostbyname(HOST_NAME)
TCP_PORT = 5100
BUFFER_SIZE = 5


clients = []
clients_lock = threading.Lock()

# ==============================
# Main Server
# ==============================

def main():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(("0.0.0.0", TCP_PORT))
    server.listen()

    print(f"Server startet at {TCP_IP}:{TCP_PORT} name: {HOST_NAME}")

    while True:
        try:
            client_socket, client_address = server.accept()
            client_socket.setblocking(True)
        
            client = Clienthandler(client_socket, client_address)

            with clients_lock:
                clients.append(client)

        except Exception as e:
            print(f"Error: e")


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
        print(f"New client: {self.client_address}")
        while self.running:
            try:
                # Zuerst status und length lesen (3 Bytes)
                header = self.client_socket.recv(3)
                if not header or len(header) < 3:
                    print(f"Unvollständiger Header von {self.client_address}.")
                    break

                print(list(header))

                status = header[0]
                length = (header[1] << 8) | header[2]

                print(f"Empfange Paket: status={status}, length={length} von {self.client_address}")

                # Dann die signal-Daten lesen (length Bytes)

                #signal_data = b""
                #while len(signal_data) < length:
                #    chunk = sock.recv(length - len(signal_data))
                #signal_data += chunk

                signal_data = self.client_socket.recv(length)
                if not signal_data or len(signal_data) < length:
                    print(f"Unvollständige Payload von {self.client_address}. Erwartet: {length}, erhalten: {len(signal_data)}")
                    break

                print(f"Paket erfolgreich empfangen: {signal_data}")

                # Paket zusammenbauen (status, length, signal_data)
                packet = {"status": status, "length": length, "signal": signal_data}
                print(f"Received packet: {packet} from: {self.client_address}")

                # Paket an alle anderen Clients weiterleiten
                broadcast(packet, self)

            except Exception as e:
                print(f"Error while receiving: {e} with: {self.client_address}")

        print(f"Client disconnected: {self.client_address}")
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
        packet = struct.pack("!BBB", status, length) + signal_data
        self.out_queue.put(packet)

    def send_loop(self):
        while self.running:
            try:
                packet = self.out_queue.get()
                if packet is None:
                    break
                self.client_socket.sendall(packet)
            except Exception as e:
                print(f"Error while sending: {e} with: {self.client_address}")


# ==============================
# Hilfsfunktionen
# ==============================

def broadcast (message, sender):
    with clients_lock:
        for client in clients:
            if client != sender:
                client.send(message)

# ==============================
# Start
# ==============================

if __name__ == "__main__":
    main()