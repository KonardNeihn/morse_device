import socket
import threading
import queue

HOST_NAME = socket.gethostname()
TCP_IP = socket.gethostbyname(HOST_NAME)
TCP_PORT = 6969
BUFFER_SIZE = 64


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
            client_socket.setblocking(False)
        
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


    def receive_loop (self):
        print(f"new client: {self.client_address}")
        try:
            while True:
                data = self.client_socket.recv(BUFFER_SIZE)

                if not data:
                    break

                message = data #data.decode()
                print(f"Received: {message} from: {self.client_address}")

                broadcast(message, self)
        
        except Exception as e:
            print(f"Error while receiving: {e} with: {self.client_address}")

        finally:
            print(f"Client disconnected: {self.client_address}")
            self.client_socket.close()
            self.running = False
            self.out_queue.put(None)
            with clients_lock:
                clients.remove(self)
        

    def send (self, message):
        self.out_queue.put(message)

    def send_loop(self):
        while self.running:
            try:
                message = self.out_queue.get()

                if message is None:
                    break

                self.client_socket.send(message.encode()) #message.encode()
            except:
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