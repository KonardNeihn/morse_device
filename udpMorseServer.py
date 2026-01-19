import socket
import threading
import queue
import time

INACTIVITY_TIMEOUT = 3600 # nach einer stunde bitte raus aus verteilerliste

messages = queue.Queue()    # Queue ist race condition safe (eingebautes mutex lock)
clients = {}    # dictionary nach dem muster (address: last_seen)
clients_lock = threading.Lock()     # lock, für das dict

server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server.bind(("192.168.178.186", 420))

def receive():
    while True:
        try:
            message, address = server.recvfrom(8)  # ein packet der maximalen länge 10 byte empfangen, blockiert den thread, bis was kommt
            messages.put((message, address))    # nachricht und herkunft speichern (thread safe)
            print(f"{message} received from {address} {time.time()}")
            with clients_lock:  # clients nicht thread sicher
                clients.update({address: time.time()}) # client liste mit zuletzt gesehener zeit
        except Exception as e:
            print(f"Error in receive {e}")

def broadcast():
    while True:
        try:
            message, address = messages.get()   # wartet solange bis ein element drin ist. blockiert die queue nicht!
            to_remove = []  # muss zwischen gespeichert werden, weil dictionary kann nicht bearbeitet werdem während iteration
            with clients_lock: # lock nicht thread sicher
                for client, last_seen in clients.items():
                    if time.time() - last_seen > INACTIVITY_TIMEOUT:  # eine stunde offline?
                        to_remove.append(client)
                    else:
                        if client != address:   # nicht an sich selber senden bitte
                            server.sendto(message, client)
                for client in to_remove:    # alle clients aus dictionary entfernen, die inaktiv waren
                    del clients[client]
        except Exception as e:
            print(f"Error in broadcast {e}")

t1 = threading.Thread(target=receive) # daemon wird mit geschlossen, wenn hauptprogramm beendet wird
t2 = threading.Thread(target=broadcast)

t1.start()
t2.start()
