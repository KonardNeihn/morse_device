import socket
import threading
import time
import keyboard

TCP_IP = "127.0.1.1"
TCP_PORT = 6969


def send(sock, msg):
    try:
        sock.send(msg.encode())
    except:
        pass


def key_loop(sock):
    while True:
        # alle 20ms checken
        if keyboard.is_pressed("space"):
            send(sock, "-")
            time.sleep(0.2)  # debounce, sonst spammt es

        time.sleep(0.02)


def receive_loop(sock):
    while True:
        try:
            data = sock.recv(1024)
            if not data:
                break
            print("\nServer:", data.decode())
        except:
            break


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((TCP_IP, TCP_PORT))

    print("Connected. Hold SPACE to send '-'")

    threading.Thread(target=receive_loop, args=(sock,), daemon=True).start()
    threading.Thread(target=key_loop, args=(sock,), daemon=True).start()

    while True:
        time.sleep(1)


if __name__ == "__main__":
    main()