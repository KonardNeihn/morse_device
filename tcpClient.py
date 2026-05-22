import socket
import threading

TCP_IP = "127.0.1.1"   # lokal testen
TCP_PORT = 6969


def receive_loop(sock):
    while True:
        try:
            data = sock.recv(1024)

            if not data:
                print("Server disconnected")
                break

            print("\nServer:", data.decode())

        except Exception as e:
            print("Receive error:", e)
            break


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((TCP_IP, TCP_PORT))

    print("Connected to server.")
    print("Type messages and press ENTER.\n")

    # Thread für empfangen
    thread = threading.Thread(target=receive_loop, args=(sock,))
    thread.daemon = True
    thread.start()

    # Hauptthread: senden
    while True:
        msg = input("> ")

        if msg.lower() == "exit":
            break

        try:
            sock.send(msg.encode())
        except:
            break

    sock.close()
    print("Disconnected.")


if __name__ == "__main__":
    main()

