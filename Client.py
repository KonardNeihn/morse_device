#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Client.py – Morse-Client für den Computer (tkinter-GUI)

Sendet Morse-Nachrichten an den Morse-Server (Server.py). Als "Morse-Taste"
dient die Leertaste (solange das Fenster fokussiert ist) oder der große Button
(Maus gedrückt halten). Host und Port werden in der GUI eingestellt.

Drahtformat exakt wie beim ESP32-Client und Server.py:
  11-Byte-Header = status (1 B) | msg_id (8 B, Big-Endian) | size (2 B, BE)
  Payload        = Bytes, jedes Byte = 8 Abtastungen (Bit gesetzt = gedrückt)

Abtastrate 10 ms, nach 5 s Ruhe gilt die Eingabe als fertig (wie in
esp32-client/include/config.h). Ton + Lampe folgen dem Signal beim Senden
und bei der Wiedergabe. Keine externen Python-Pakete nötig.
"""

import array
import io
import math
import os
import queue
import shutil
import socket
import struct
import subprocess
import tempfile
import threading
import time
import tkinter as tk
import uuid
import wave
from tkinter import ttk

# ---------------------------------------------------------------------------
# Konfiguration (passend zu Server.py / esp32-client/include/config.h)
# ---------------------------------------------------------------------------
DEFAULT_HOST = "morse-server.de"
DEFAULT_PORT = 6969

SAMPLING_RATE_MS = 10        # eine Abtastung der "Taste" alle x ms
RECORDING_TIMEOUT_MS = 5000  # nach so langer Pause gilt die Eingabe als fertig
KEEPALIVE_INTERVAL_S = 10    # Abstand der App-Keepalive-Pakete
CONNECTION_TIMEOUT_S = 30    # Fallback-Timeout, falls TCP-Keepalive nicht greift
TCP_KEEPALIVE_IDLE_S = 10    # TCP-Keepalive: Ruhe vor der 1. Probe (wie ESP32)
TCP_KEEPALIVE_INTERVAL_S = 5 # TCP-Keepalive: Abstand der Proben
TCP_KEEPALIVE_COUNT = 3      # TCP-Keepalive: fehlgeschlagene Proben bis "tot"
RECONNECT_DELAY_S = 3        # Wartezeit nach Verbindungsverlust
SOUND_FREQ = 700             # Morse-Sidetone in Hz (üblich: 600–1000 Hz)

STATUS_KEEPALIVE = 0
STATUS_MESSAGE = 1
STATUS_ACK = 2
STATUS_CHECK = 3
STATUS_REGISTER = 4

HEADER = struct.Struct("!BQH")


def get_mac():
    """6-Byte-MAC als Identität gegenüber dem Server."""
    node = uuid.getnode()
    if node == 0:
        node = uuid.uuid4().int & ((1 << 48) - 1)
    return node.to_bytes(6, "big")


def payload_to_bits(payload):
    """Payload als 0/1-String ausgeben (MSB zuerst, je Byte)."""
    return "".join("{:08b}".format(b) for b in payload)


def _preview(text, limit=96):
    return text if len(text) <= limit else text[:limit] + "..."


def _bits_of(payload):
    """Payload als Liste von Bits (MSB zuerst, je Byte)."""
    bits = []
    for b in payload:
        for i in range(7, -1, -1):
            bits.append((b >> i) & 1)
    return bits


class MorseSampler:
    """Baut aus Taste-Abtastungen den Morse-Bytestream (wie input_task.cpp)."""

    def __init__(self, timeout_ms=RECORDING_TIMEOUT_MS):
        self.timeout_ms = timeout_ms
        self.reset()

    def reset(self):
        self.recording = False
        self.current_bytes = []
        self.sample_buffer = []
        self.last_pressed = 0
        self.start_ms = 0
        self.sample_count = 0

    def tick(self, now_ms, pressed):
        if pressed and not self.recording:
            self.recording = True
            self.start_ms = now_ms
            self.current_bytes = []
            self.sample_buffer = []
            self.sample_count = 0
        if pressed:
            self.last_pressed = now_ms
        if not self.recording:
            return None

        self.sample_count += 1
        self.sample_buffer.append(1 if pressed else 0)
        if len(self.sample_buffer) == 8:
            byte = 0
            for b in self.sample_buffer:
                byte = (byte << 1) | b
            self.current_bytes.append(byte)
            self.sample_buffer = []

        if (now_ms - self.last_pressed) >= self.timeout_ms:
            payload = bytes(self.current_bytes)
            self.recording = False
            self.sample_buffer = []
            self.current_bytes = []
            return payload or None
        return None

    def current_bits(self):
        """Alle bisherigen Abtastungen der laufenden Aufnahme als Bit-Liste."""
        bits = []
        for b in self.current_bytes:
            for i in range(7, -1, -1):
                bits.append((b >> i) & 1)
        bits.extend(self.sample_buffer)
        return bits


# ---------------------------------------------------------------------------
# Ton: bevorzugt pygame (niedrige Latenz), sonst paplay/pw-play/aplay
# ---------------------------------------------------------------------------
class SoundEngine:
    """Morse-Töne (SOUND_FREQ) erzeugen und abspielen.

    Bevorzugt pygame.mixer (niedrige Latenz -> Sidetone auch bei sehr kurzen
    Signalen). Ohne pygame fällt es auf paplay/pw-play/aplay zurück (höhere
    Latenz durch Prozessstart).
    """

    def __init__(self, freq=SOUND_FREQ, bit_ms=SAMPLING_RATE_MS):
        self.freq = freq
        self.bit_ms = bit_ms
        self.rate = 22050
        self._pg = self._init_pygame()
        self._cmd = None if self._pg is not None else self._detect_player()
        self._player = None            # subprocess (Fallback)
        self._tone_file = None
        self._last_play_file = None
        self._tone = None              # pygame Sound (Dauerton)
        self._tone_channel = None
        self._msg_sound = None
        self._msg_channel = None

    def _init_pygame(self):
        try:
            import pygame
            pygame.mixer.init(frequency=22050, size=-16, channels=1, buffer=256)
            # Smoke-Test, damit wir sicher wissen, dass das Backend funktioniert
            pygame.mixer.Sound(buffer=array.array("h", [0] * 64))
            return pygame
        except Exception:
            return None

    def _detect_player(self):
        for name in ("paplay", "pw-play", "aplay"):
            path = shutil.which(name)
            if path:
                return (name, path)
        return None

    @property
    def available(self):
        return self._pg is not None or self._cmd is not None

    # -- Samples erzeugen -----------------------------------------------------
    def _render(self, on_flags):
        out = array.array("h")
        t = 0
        for on in on_flags:
            if on:
                v = int(0.6 * 32767 * math.sin(2 * math.pi * self.freq * t / self.rate))
            else:
                v = 0
            out.append(v)
            t += 1
        return out

    def _wrap_wav(self, samples):
        buf = io.BytesIO()
        w = wave.open(buf, "wb")
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(self.rate)
        w.writeframes(samples.tobytes())
        w.close()
        return buf.getvalue()

    def _tone_samples(self):
        # 1 s Dauerton (ganzzahlig viele Perioden -> nahtlose Endlos-Schleife)
        return self._render([True] * self.rate)

    def _message_samples(self, payload):
        per_bit = max(1, int(round(self.rate * self.bit_ms / 1000.0)))
        flags = []
        for b in payload:
            for i in range(7, -1, -1):
                flags += [bool((b >> i) & 1)] * per_bit
        return self._render(flags)

    # -- Steuerung ------------------------------------------------------------
    def start_tone(self):
        if self._pg is not None:
            if self._tone is None:
                self._tone = self._pg.mixer.Sound(buffer=self._tone_samples())
            if self._tone_channel is not None:
                self._tone_channel.stop()
            self._tone_channel = self._tone.play(loops=-1)
        elif self._cmd is not None:
            if self._tone_file is None:
                fd, path = tempfile.mkstemp(suffix=".wav")
                with os.fdopen(fd, "wb") as f:
                    f.write(self._wrap_wav(self._render([True] * (self.rate * 10))))
                self._tone_file = path
            self._launch(self._tone_file)

    def stop_tone(self):
        if self._pg is not None:
            if self._tone_channel is not None:
                self._tone_channel.stop()
                self._tone_channel = None
        elif self._player is not None:
            try:
                self._player.terminate()
            except OSError:
                pass
            self._player = None

    def play_bits(self, payload):
        if self._pg is not None:
            self.stop()
            self._msg_sound = self._pg.mixer.Sound(buffer=self._message_samples(payload))
            self._msg_channel = self._msg_sound.play()
        elif self._cmd is not None:
            self._cleanup_play_file()
            fd, path = tempfile.mkstemp(suffix=".wav")
            with os.fdopen(fd, "wb") as f:
                f.write(self._wrap_wav(self._message_samples(payload)))
            self._last_play_file = path
            self._launch(path)

    def stop(self):
        if self._pg is not None:
            self.stop_tone()
            if self._msg_channel is not None:
                self._msg_channel.stop()
                self._msg_channel = None
        else:
            self.stop_tone()

    # -- Fallback (Subprozess) -------------------------------------------------
    def _launch(self, wav_path):
        self.stop_tone()
        if self._cmd is None:
            return
        name, path = self._cmd
        cmd = [path] + (["-q"] if name == "aplay" else []) + [wav_path]
        try:
            self._player = subprocess.Popen(
                cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except OSError:
            self._player = None

    def _cleanup_play_file(self):
        if self._last_play_file is not None:
            try:
                os.unlink(self._last_play_file)
            except OSError:
                pass
            self._last_play_file = None


# ---------------------------------------------------------------------------
# Netzwerk-Schicht: verbinden, registrieren, senden/empfangen
# ---------------------------------------------------------------------------
class MorseNet:
    """TCP-Verbindung zum Server in einem eigenen Thread.

    Ereignisse (in die thread-sichere `events`-Queue):
      ("status", text)         – Verbindungsstatus
      ("error", text)          – Fehlermeldung
      ("sent", mid, payload)   – Nachricht in die Sende-Queue gelegt
      ("ack", mid)             – Annahme-ACK vom Server
      ("recv", mid, payload)   – empfangene Nachricht
      ("delivery_ack", mid)    – Zustell-ACK wurde gesendet
      ("keepalive_sent",)      – Keepalive gesendet
      ("keepalive_reply",)     – Keepalive-Antwort empfangen
      ("disconnected",)        – Verbindung beendet
      ("log", text)            – sonstiges
    """

    def __init__(self, events):
        self.events = events
        self.mac = get_mac()
        self.send_queue = queue.Queue()
        self.running = False
        self.sock = None
        self.thread = None
        self._recv_buffer = b""
        self._next_msg_id = 0
        self._msg_id_lock = threading.Lock()
        self._target_lock = threading.Lock()
        self._host = DEFAULT_HOST
        self._port = DEFAULT_PORT
        self._wake = threading.Event()
        self._last_recv = time.monotonic()   # letzter Empfang (Tot-Erkennung)
        self._pending_sends = {}             # client_msg_id -> payload (noch nicht bestätigt)
        self._pending_lock = threading.Lock()

    def set_target(self, host, port):
        with self._target_lock:
            self._host = host
            self._port = port

    def get_target(self):
        with self._target_lock:
            return self._host, self._port

    def reconnect(self):
        """Verbindung neu aufbauen (weckt ggf. eine laufende Wartezeit auf)."""
        self._wake.set()
        sock = self.sock
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._network_loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        self._wake.set()
        sock = self.sock
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass
        self.send_queue.put(None)
        if self.thread is not None:
            self.thread.join(timeout=2)

    def send_message(self, payload):
        """Morse-Payload mit neuer client_msg_id in die Sendeschlange legen.

        Die msg_id ist zeitbasiert (Nanosekunden), damit sie auch nach einem
        Neustart des Clients NICHT mit früheren msg_ids kollidiert. Sonst würde
        der Server die Nachricht als Retry einstufen und nicht puffern/broadcasten.
        """
        with self._msg_id_lock:
            mid = time.time_ns()
            if mid <= self._next_msg_id:
                mid = self._next_msg_id + 1
            self._next_msg_id = mid
        self.send_queue.put(HEADER.pack(STATUS_MESSAGE, mid, len(payload)) + payload)
        with self._pending_lock:
            self._pending_sends[mid] = payload
        self.events.put(("sent", mid, payload))

    # -- intern -------------------------------------------------------------
    def _emit(self, kind, *args):
        self.events.put((kind,) + args)

    def _sleep(self, seconds):
        self._wake.wait(seconds)
        self._wake.clear()

    def _drain_queue(self):
        while True:
            try:
                self.send_queue.get_nowait()
            except queue.Empty:
                break

    def _close_sock(self, sock):
        try:
            sock.close()
        except OSError:
            pass

    def _requeue_pending(self):
        """Noch nicht bestätigte Nachrichten nach einem Reconnect erneut senden."""
        with self._pending_lock:
            for mid, payload in self._pending_sends.items():
                self.send_queue.put(HEADER.pack(STATUS_MESSAGE, mid, len(payload)) + payload)


    def _network_loop(self):
        while self.running:
            host, port = self.get_target()
            self._emit("status", "Verbinde mit %s:%d ..." % (host, port))
            sock = self._connect(host, port)
            if sock is None:
                continue
            self.sock = sock

            self._emit("status", "Registriere ...")
            try:
                sock.sendall(HEADER.pack(STATUS_REGISTER, 0, len(self.mac)) + self.mac)
            except OSError as e:
                self._emit("error", "Registrierung fehlgeschlagen: %s" % e)
                self._emit("status", "Registrierung fehlgeschlagen")
                self._close_sock(sock)
                self.sock = None
                self._sleep(RECONNECT_DELAY_S)
                continue

            self._drain_queue()
            self._requeue_pending()
            self._recv_buffer = b""
            self._last_recv = time.monotonic()
            self._emit("status", "Verbunden – bereit zum Morsen")
            self._serve(sock)

            self._close_sock(sock)
            self.sock = None
            self._emit("disconnected",)
            self._emit("status", "Verbindung getrennt – verbinde neu ...")
            self._sleep(RECONNECT_DELAY_S)

    def _connect(self, host, port):
        try:
            sock = socket.create_connection((host, port), timeout=10)
        except OSError as e:
            self._emit("error", "Verbindung fehlgeschlagen: %s" % e)
            self._emit("status", "Verbindung fehlgeschlagen")
            self._sleep(RECONNECT_DELAY_S)
            return None
        self._enable_tcp_keepalive(sock)
        return sock

    def _enable_tcp_keepalive(self, sock):
        """TCP-Keepalive aktivieren, damit das OS tote Verbindungen selbst erkennt.

        Werte wie beim ESP32 (TCP_KEEPALIVE_* in dessen config.h). Die
        Feineinstellung (KEEPIDLE/INTVL/CNT) ist Linux-spezifisch; wo sie
        fehlt, gilt der OS-Standard (~2 h) – dann greift CONNECTION_TIMEOUT_S.
        """
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        except OSError:
            return
        if hasattr(socket, "TCP_KEEPIDLE"):
            try:
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, TCP_KEEPALIVE_IDLE_S)
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, TCP_KEEPALIVE_INTERVAL_S)
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, TCP_KEEPALIVE_COUNT)
            except OSError:
                pass

    def _serve(self, sock):
        sock.settimeout(0.05)
        last_keepalive = time.monotonic()
        while self.running:
            try:
                while True:
                    packet = self.send_queue.get_nowait()
                    if packet is None:
                        return
                    sock.sendall(packet)
            except queue.Empty:
                pass
            except OSError:
                return

            now = time.monotonic()

            # Tote Verbindung erkennen: keine Daten (auch kein Keepalive-Echo)
            # mehr empfangen, obwohl der Server auf jedes Keepalive antwortet.
            if now - self._last_recv > CONNECTION_TIMEOUT_S:
                self._emit("status", "Kein Lebenszeichen vom Server – trenne")
                return

            if now - last_keepalive >= KEEPALIVE_INTERVAL_S:
                try:
                    sock.sendall(HEADER.pack(STATUS_KEEPALIVE, 0, 0))
                except OSError:
                    return
                last_keepalive = now
                self._emit("keepalive_sent",)

            try:
                data = sock.recv(65536)
            except socket.timeout:
                continue
            except OSError:
                return
            if not data:
                return
            self._last_recv = time.monotonic()
            self._recv_buffer += data
            self._process_buffer(sock)


    def _process_buffer(self, sock):
        while True:
            if len(self._recv_buffer) < HEADER.size:
                return
            status, msg_id, length = HEADER.unpack(self._recv_buffer[:HEADER.size])
            if len(self._recv_buffer) < HEADER.size + length:
                return  # Paket noch nicht vollständig
            payload = self._recv_buffer[HEADER.size:HEADER.size + length]
            self._recv_buffer = self._recv_buffer[HEADER.size + length:]
            self._handle_packet(sock, status, msg_id, payload)

    def _handle_packet(self, sock, status, msg_id, payload):
        if status == STATUS_KEEPALIVE:
            self._emit("keepalive_reply",)
            return
        if status == STATUS_ACK:
            with self._pending_lock:
                self._pending_sends.pop(msg_id, None)
            self._emit("ack", msg_id)
            return
        if status in (STATUS_MESSAGE, STATUS_CHECK):
            self._emit("recv", msg_id, payload)
            try:
                sock.sendall(HEADER.pack(STATUS_ACK, msg_id, 0))
                self._emit("delivery_ack", msg_id)
            except OSError:
                pass
            return
        self._emit("log", "Unbekannter Status %d (msg_id=%d)" % (status, msg_id))


# ---------------------------------------------------------------------------
# GUI
# ---------------------------------------------------------------------------
class MorseApp:
    DOT = 2          # Pixel pro Druckpunkt
    DIVIDER = 1      # Höhe der Trennlinie zwischen den beiden Reihen
    ROW_GAP = 2      # Abstand um die Trennlinie
    LINE_GAP = 6     # Abstand zwischen zwei gedruckten Zeilen
    LINE_H = DOT + ROW_GAP + DIVIDER + ROW_GAP + DOT        # Höhe einer Zeile
    LINE_ADVANCE = LINE_H + LINE_GAP                         # Zeilenanfang zu Zeilenanfang

    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Morse-Client")
        self.root.geometry("1100x800")
        self.root.minsize(860, 600)

        self.events = queue.Queue()
        self.net = MorseNet(self.events)
        self.sampler = MorseSampler()
        self.sound = SoundEngine()

        self.key_down = False
        self._key_was_down = False
        self.playing = False
        self._playback_bits = []
        self._playback_idx = 0
        # Zustand für die fortlaufende Drucker-Vorschau (append-only, flüssig)
        self._preview_idx = 0
        self._preview_writing_top = True
        self._preview_y = 4
        self._run_item = None      # Canvas-Objekt des aktuellen Punktlaufs
        self._run_x0 = 0
        self._run_y = 0
        self._drawn_bits = 0       # bereits gezeichnete Bits der laufenden Aufnahme
        self._page_empty = True    # noch nichts gezeichnet
        self._pending_display = [] # Empfänge, die während der Aufnahme warten

        # Zähler / Debug-Werte
        self.sent_count = 0
        self.ack_count = 0
        self.recv_count = 0
        self.delivery_ack_count = 0
        self.reconnect_count = 0
        self.last_client_msg_id = 0
        self.last_server_msg_id = 0
        self.last_keepalive_sent_ts = None
        self.last_keepalive_reply_ts = None
        self.last_error = "–"

        self._build_ui()

    def run(self):
        self._apply_target()
        self.net.start()
        self._poll_events()
        self._sample_tick()
        self.root.mainloop()

    # -- UI -----------------------------------------------------------------
    def _build_ui(self):
        # Konfigurationszeile: Host / Port / Verbinden / Status
        top = tk.Frame(self.root)
        top.pack(fill=tk.X, padx=8, pady=(8, 4))
        tk.Label(top, text="Host:").pack(side=tk.LEFT)
        self.host_var = tk.StringVar(value=DEFAULT_HOST)
        self.host_entry = tk.Entry(top, textvariable=self.host_var, width=20)
        self.host_entry.pack(side=tk.LEFT, padx=(2, 8))
        tk.Label(top, text="Port:").pack(side=tk.LEFT)
        self.port_var = tk.StringVar(value=str(DEFAULT_PORT))
        self.port_entry = tk.Entry(top, textvariable=self.port_var, width=6)
        self.port_entry.pack(side=tk.LEFT, padx=(2, 8))
        # Leertaste in den Eingabefeldern nicht als Zeichen einfügen,
        # sondern als Morse-Taste verwenden.
        self.host_entry.bind("<KeyPress-space>", self._on_key_press)
        self.port_entry.bind("<KeyPress-space>", self._on_key_press)
        self.connect_btn = tk.Button(top, text="Verbinden", command=self._on_connect)
        self.connect_btn.pack(side=tk.LEFT)
        self.status_var = tk.StringVar(value="Starte ...")
        tk.Label(top, textvariable=self.status_var, anchor="w", fg="#036").pack(
            side=tk.LEFT, padx=12)

        # Mitte: links Aufnahme-Monitor, rechts Lampe + Drucker-Vorschau
        mid = tk.Frame(self.root)
        mid.pack(fill=tk.BOTH, expand=True, padx=8, pady=4)
        mid.columnconfigure(1, weight=1)
        mid.rowconfigure(0, weight=1)

        left = tk.LabelFrame(mid, text="Aufnahme")
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 4))
        self._build_recording_monitor(left)

        right = tk.LabelFrame(mid, text="Signal (Thermodrucker-Ansicht)")
        right.grid(row=0, column=1, sticky="nsew")
        self._build_signal_view(right)

        # Debug-Panel
        debug = tk.LabelFrame(self.root, text="Debug / Überwachung")
        debug.pack(fill=tk.X, padx=8, pady=4)
        self._build_debug(debug)

        # Nachrichten-Tabelle
        table_frame = tk.LabelFrame(self.root, text="Nachrichten")
        table_frame.pack(fill=tk.BOTH, expand=True, padx=8, pady=4)
        self._build_table(table_frame)

        # Log
        log_frame = tk.LabelFrame(self.root, text="Log")
        log_frame.pack(fill=tk.X, padx=8, pady=(4, 8))
        self.log_text = tk.Text(log_frame, height=5, state=tk.DISABLED, font=("Courier", 9))
        self.log_vbar = tk.Scrollbar(log_frame, orient=tk.VERTICAL,
                                     command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=self.log_vbar.set)
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.log_vbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_text.bind("<Configure>", lambda e: self._refresh_log_scrollbar())

        # Eingabe-Bindings
        self.root.bind_all("<KeyPress-space>", self._on_key_press)
        self.root.bind_all("<KeyRelease-space>", self._on_key_release)
        self.root.bind_all("<ButtonRelease-1>", self._on_button_release)
        # Klick irgendwo ins Fenster -> Fokus von Eingabefeldern nehmen.
        self.root.bind("<Button-1>", self._on_root_click)


    def _build_recording_monitor(self, parent):
        self.rec_state_var = tk.StringVar(value="bereit")
        self.elapsed_var = tk.StringVar(value="0.00 s")
        self.remaining_var = tk.StringVar(value="%.2f s" % (RECORDING_TIMEOUT_MS / 1000.0))
        self.samples_var = tk.StringVar(value="0")
        self.bytes_var = tk.StringVar(value="0")

        rows = [
            ("Zustand", self.rec_state_var),
            ("Verstrichen", self.elapsed_var),
            ("Verbleibend", self.remaining_var),
            ("Samples", self.samples_var),
            ("Bytes", self.bytes_var),
        ]
        for r, (label, var) in enumerate(rows):
            tk.Label(parent, text=label + ":", anchor="w").grid(
                row=r, column=0, sticky="w", padx=6, pady=1)
            tk.Label(parent, textvariable=var, anchor="e",
                     font=("Helvetica", 11, "bold")).grid(
                row=r, column=1, sticky="e", padx=6, pady=1)

        self.button = tk.Button(parent, text="MORSE  (gedrückt halten)",
                                font=("Helvetica", 14, "bold"), bg="#3a6",
                                activebackground="#2c5", height=2, takefocus=False)
        self.button.grid(row=len(rows), column=0, columnspan=2, sticky="ew", padx=6, pady=8)
        self.button.bind("<ButtonPress-1>", self._on_button_press)

        tk.Label(parent, text="Leertaste halten oder Button\ngedrückt halten.\n5 s Pause = senden.",
                 anchor="w", justify=tk.LEFT, fg="#555").grid(
            row=len(rows) + 1, column=0, columnspan=2, sticky="w", padx=6)

    def _build_signal_view(self, parent):
        lamp_frame = tk.Frame(parent)
        lamp_frame.pack(fill=tk.X, padx=6, pady=4)
        self.lamp_canvas = tk.Canvas(lamp_frame, width=36, height=36, bg="#222",
                                     highlightthickness=0)
        self.lamp_canvas.pack(side=tk.LEFT)
        self._lamp_id = self.lamp_canvas.create_oval(6, 6, 30, 30, fill="#444", outline="#000")
        tk.Label(lamp_frame, text="Lampe").pack(side=tk.LEFT, padx=8)

        cv_frame = tk.Frame(parent)
        cv_frame.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)
        self.signal_canvas = tk.Canvas(cv_frame, bg="white", highlightthickness=1,
                                       highlightbackground="#aaa")
        self.signal_hbar = tk.Scrollbar(cv_frame, orient=tk.HORIZONTAL,
                                        command=self.signal_canvas.xview)
        self.signal_vbar = tk.Scrollbar(cv_frame, orient=tk.VERTICAL,
                                        command=self.signal_canvas.yview)
        self.signal_canvas.configure(xscrollcommand=self.signal_hbar.set,
                                     yscrollcommand=self.signal_vbar.set)
        self.signal_canvas.grid(row=0, column=0, sticky="nsew")
        self.signal_vbar.grid(row=0, column=1, sticky="ns")
        self.signal_hbar.grid(row=1, column=0, sticky="ew")
        # Scrollbalken nur anzeigen, wenn wirklich gescrollt werden kann
        self.signal_canvas.bind("<Configure>", lambda e: self._refresh_scrollbars())
        cv_frame.rowconfigure(0, weight=1)
        cv_frame.columnconfigure(0, weight=1)

    def _build_debug(self, parent):
        self.mac_var = tk.StringVar(value="–")
        self.keepalive_sent_var = tk.StringVar(value="–")
        self.keepalive_reply_var = tk.StringVar(value="–")
        self.next_msgid_var = tk.StringVar(value="1")
        self.last_server_msgid_var = tk.StringVar(value="–")
        self.counters_var = tk.StringVar(value="")
        self.queue_var = tk.StringVar(value="0")
        self.reconnect_var = tk.StringVar(value="0")
        self.error_var = tk.StringVar(value="–")

        def cell(r, c, label, var):
            f = tk.Frame(parent)
            f.grid(row=r, column=c, sticky="w", padx=6, pady=1)
            tk.Label(f, text=label + ":", anchor="w").pack(side=tk.LEFT)
            tk.Label(f, textvariable=var, anchor="w").pack(side=tk.LEFT, padx=4)

        cell(0, 0, "MAC", self.mac_var)
        cell(0, 1, "Keepalive gesendet (vor)", self.keepalive_sent_var)
        cell(0, 2, "Keepalive Antwort (vor)", self.keepalive_reply_var)
        cell(1, 0, "Nächste client_msg_id", self.next_msgid_var)
        cell(1, 1, "Letzte server_msg_id", self.last_server_msgid_var)
        cell(1, 2, "Sendewarteschlange", self.queue_var)
        cell(2, 0, "Nachrichten", self.counters_var)
        cell(2, 1, "Reconnects", self.reconnect_var)
        cell(3, 0, "Letzter Fehler", self.error_var)

    def _build_table(self, parent):
        cols = ("time", "dir", "msgid", "bytes", "status", "signal")
        self.tree = ttk.Treeview(parent, columns=cols, show="headings", height=7)
        headings = {"time": "Zeit", "dir": "Richtung", "msgid": "msg_id",
                    "bytes": "Bytes", "status": "Status", "signal": "Signal"}
        widths = {"time": 80, "dir": 70, "msgid": 70, "bytes": 50,
                  "status": 110, "signal": 240}
        for c in cols:
            self.tree.heading(c, text=headings[c])
            anchor = "w" if c in ("status", "signal") else "center"
            self.tree.column(c, width=widths[c], anchor=anchor, stretch=(c == "signal"))
        self.tree_vbar = tk.Scrollbar(parent, orient=tk.VERTICAL, command=self.tree.yview)
        self.tree.configure(yscrollcommand=self.tree_vbar.set)
        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.tree_vbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.tree.bind("<Configure>", lambda e: self._refresh_table_scrollbar())


    # -- Eingabe -------------------------------------------------------------
    def _on_button_press(self, event):
        self.key_down = True
        self.button.config(relief=tk.SUNKEN)

    def _on_button_release(self, event):
        self.key_down = False
        self.button.config(relief=tk.RAISED)

    def _on_key_press(self, event):
        self.key_down = True
        return "break"  # verhindert z. B. Leerzeichen in den Eingabefeldern

    def _on_root_click(self, event):
        # Klick außerhalb der Eingabefelder -> Fokus vom Feld nehmen (deselecten)
        if not isinstance(event.widget, tk.Entry):
            self.root.focus_set()

    def _on_key_release(self, event):
        self.key_down = False

    def _set_lamp(self, on):
        self.lamp_canvas.itemconfig(self._lamp_id, fill="#fd0" if on else "#444")

    def _apply_target(self):
        host = self.host_var.get().strip() or DEFAULT_HOST
        try:
            port = int(self.port_var.get().strip())
        except ValueError:
            port = DEFAULT_PORT
        self.net.set_target(host, port)
        return host, port

    def _on_connect(self):
        host, port = self._apply_target()
        self._log("Verbinde mit %s:%d ..." % (host, port))
        self.net.reconnect()

    # -- Morse-Aufnahme (10-ms-Takt) ----------------------------------------
    def _sample_tick(self):
        now_ms = int(time.monotonic() * 1000)
        pressed = self.key_down
        was_recording = self.sampler.recording

        if pressed and not self._key_was_down:
            self.sound.start_tone()
        elif not pressed and self._key_was_down:
            self.sound.stop_tone()
        self._key_was_down = pressed

        if pressed and self.playing:
            self._stop_playback()

        # Start einer neuen Aufnahme -> Separator (Zeilenumbruch + Leerzeile)
        if pressed and not was_recording and not self.playing:
            self._start_new_message()
            self._drawn_bits = 0

        payload = self.sampler.tick(now_ms, pressed)

        # Live-Vorschau: nur die neuen Bits anhängen (ein Bit pro Tick -> flüssig)
        if self.sampler.recording and not self.playing:
            bits = self.sampler.current_bits()
            self._append_bits(bits[self._drawn_bits:])
            self._drawn_bits = len(bits)

        if payload is not None:
            self.net.send_message(payload)
            # Während der Aufnahme zurückgehaltene Empfänge jetzt anzeigen/abspielen
            while self._pending_display:
                p = self._pending_display.pop(0)
                self._append_message(p)
                self._play_message(p)

        if self.sampler.recording and not self.playing:
            self._set_lamp(pressed)

        self.root.after(SAMPLING_RATE_MS, self._sample_tick)

    # -- Wiedergabe ----------------------------------------------------------
    def _play_message(self, payload):
        bits = []
        for b in payload:
            for i in range(7, -1, -1):
                bits.append((b >> i) & 1)
        if not bits:
            return
        self._stop_playback()
        self.sound.play_bits(payload)
        self.playing = True
        self._playback_bits = bits
        self._playback_idx = 0
        self._playback_step()

    def _playback_step(self):
        if not self.playing or self._playback_idx >= len(self._playback_bits):
            self.playing = False
            self._set_lamp(False)
            return
        bit = self._playback_bits[self._playback_idx]
        self._playback_idx += 1
        self._set_lamp(bool(bit))
        self.root.after(SAMPLING_RATE_MS, self._playback_step)

    def _stop_playback(self):
        self.playing = False
        self._playback_bits = []
        self.sound.stop()
        self._set_lamp(False)


    # -- Signal-Vorschau (Thermodrucker, fortlaufend) -------------------------
    def _update_scrollregion(self):
        width = 4 + 384 * self.DOT
        self.signal_canvas.configure(
            scrollregion=(0, 0, width + 4, max(self._preview_y + self.DOT, 1)))
        self._refresh_scrollbars()

    def _refresh_scrollbars(self):
        if self.signal_canvas.xview() == (0.0, 1.0):
            self.signal_hbar.grid_remove()
        else:
            self.signal_hbar.grid()
        if self.signal_canvas.yview() == (0.0, 1.0):
            self.signal_vbar.grid_remove()
        else:
            self.signal_vbar.grid()

    def _refresh_table_scrollbar(self):
        if self.tree.yview() == (0.0, 1.0):
            self.tree_vbar.pack_forget()
        else:
            self.tree_vbar.pack(side=tk.RIGHT, fill=tk.Y)

    def _refresh_log_scrollbar(self):
        if self.log_text.yview() == (0.0, 1.0):
            self.log_vbar.pack_forget()
        else:
            self.log_vbar.pack(side=tk.RIGHT, fill=tk.Y)

    def _append_bits(self, bits):
        """Hängt Bits ans Ende an, ohne das Canvas neu aufzubauen.

        Layout wie beim Thermodrucker: obere Reihe (384 Punkte), Trennlinie,
        untere Reihe (384 Punkte). Zusammenhängende Punkte werden als EIN
        Rechteck gezeichnet (statt ein Rechteck pro Punkt) -> flüssig.
        """
        width = 4 + 384 * self.DOT
        for bit in bits:
            if bit:
                x = 4 + self._preview_idx * self.DOT
                if self._run_item is not None:
                    self.signal_canvas.coords(
                        self._run_item,
                        self._run_x0, self._run_y, x + self.DOT, self._run_y + self.DOT)
                else:
                    self._run_x0 = x
                    self._run_y = self._preview_y
                    self._run_item = self.signal_canvas.create_rectangle(
                        x, self._run_y, x + self.DOT, self._run_y + self.DOT,
                        fill="#111", outline="")
            else:
                self._run_item = None
            self._preview_idx += 1
            if self._preview_idx == 384:
                self._run_item = None  # Lauf endet am Zeilenrand
                if self._preview_writing_top:
                    # obere Reihe voll -> Trennlinie, dann untere Reihe
                    self._preview_y += self.DOT + self.ROW_GAP
                    self.signal_canvas.create_rectangle(
                        4, self._preview_y, width, self._preview_y + self.DIVIDER,
                        fill="#999", outline="")
                    self._preview_y += self.DIVIDER + self.ROW_GAP
                    self._preview_writing_top = False
                    self._preview_idx = 0
                else:
                    # Zeile voll -> nächste Zeile
                    self._preview_y += self.DOT + self.LINE_GAP
                    self._preview_writing_top = True
                    self._preview_idx = 0
        self._update_scrollregion()

    def _start_new_message(self):
        """Beginnt eine neue Nachricht: Zeilenumbruch + Leerzeile als Abstand."""
        self._run_item = None
        if not self._page_empty:
            # Laufende (Teil-)Zeile abschließen
            if self._preview_writing_top:
                if self._preview_idx != 0:
                    self._preview_y += self.LINE_ADVANCE
            else:
                self._preview_y += self.DOT + self.LINE_GAP
            # Leerzeile
            self._preview_y += self.LINE_ADVANCE
        self._preview_idx = 0
        self._preview_writing_top = True
        self._page_empty = False
        self._update_scrollregion()

    def _append_message(self, payload):
        self._start_new_message()
        self._append_bits(_bits_of(payload))

    # -- Ereignisse aus dem Netzwerk-Thread ----------------------------------
    def _poll_events(self):
        try:
            while True:
                kind, *rest = self.events.get_nowait()
                self._handle_event(kind, rest)
        except queue.Empty:
            pass
        self._update_monitor()
        self.root.after(100, self._poll_events)

    def _handle_event(self, kind, rest):
        if kind == "status":
            self.status_var.set(rest[0])
            self._log(rest[0])
        elif kind == "error":
            self.last_error = rest[0]
            self._log("FEHLER: " + rest[0])
        elif kind == "sent":
            mid, payload = rest
            self.sent_count += 1
            self.last_client_msg_id = mid
            self._table_add("→", "send", mid, payload, "gesendet")
        elif kind == "ack":
            self.ack_count += 1
            self._table_update("send", rest[0], "ACK erhalten")
        elif kind == "recv":
            mid, payload = rest
            self.recv_count += 1
            self.last_server_msg_id = mid
            self._table_add("←", "recv", mid, payload, "empfangen")
            if self.sampler.recording:
                # Während der eigenen Aufnahme zurückhalten, danach anzeigen/abspielen.
                self._pending_display.append(payload)
            else:
                self._append_message(payload)
                self._play_message(payload)
        elif kind == "delivery_ack":
            self.delivery_ack_count += 1
            self._table_update("recv", rest[0], "bestätigt")
        elif kind == "keepalive_sent":
            self.last_keepalive_sent_ts = time.monotonic()
        elif kind == "keepalive_reply":
            self.last_keepalive_reply_ts = time.monotonic()
        elif kind == "disconnected":
            self.reconnect_count += 1
        elif kind == "log":
            self._log(rest[0])

    # -- Nachrichten-Tabelle --------------------------------------------------
    def _table_add(self, direction, key_prefix, mid, payload, status):
        iid = "%s-%d" % (key_prefix, mid)
        ts = time.strftime("%H:%M:%S")
        values = (ts, direction, str(mid), str(len(payload)), status,
                  _preview(payload_to_bits(payload), 24))
        self.tree.insert("", "end", iid=iid, values=values)
        self._trim_table()
        self._refresh_table_scrollbar()

    def _table_update(self, key_prefix, mid, status):
        iid = "%s-%d" % (key_prefix, mid)
        if self.tree.exists(iid):
            values = list(self.tree.item(iid, "values"))
            values[4] = status
            self.tree.item(iid, values=values)

    def _trim_table(self, limit=200):
        children = self.tree.get_children()
        excess = len(children) - limit
        for iid in children[:excess]:
            self.tree.delete(iid)


    # -- Monitor --------------------------------------------------------------
    def _update_monitor(self):
        now_ms = int(time.monotonic() * 1000)
        if self.sampler.recording:
            self.rec_state_var.set("AUFNAHME")
            self.elapsed_var.set("%.2f s" % ((now_ms - self.sampler.start_ms) / 1000.0))
            rem = max(0, RECORDING_TIMEOUT_MS - (now_ms - self.sampler.last_pressed))
            self.remaining_var.set("%.2f s" % (rem / 1000.0))
            self.samples_var.set(str(self.sampler.sample_count))
            self.bytes_var.set(str(len(self.sampler.current_bytes)))
        else:
            self.rec_state_var.set("bereit")
            self.elapsed_var.set("0.00 s")
            self.remaining_var.set("%.2f s" % (RECORDING_TIMEOUT_MS / 1000.0))
            self.samples_var.set("0")
            self.bytes_var.set("0")

        self.mac_var.set(":".join("%02x" % b for b in self.net.mac))
        now = time.monotonic()
        self.keepalive_sent_var.set(
            ("%.1f s" % (now - self.last_keepalive_sent_ts))
            if self.last_keepalive_sent_ts is not None else "–")
        self.keepalive_reply_var.set(
            ("%.1f s" % (now - self.last_keepalive_reply_ts))
            if self.last_keepalive_reply_ts is not None else "–")
        self.next_msgid_var.set(str(self.last_client_msg_id + 1))
        self.last_server_msgid_var.set(
            str(self.last_server_msg_id) if self.last_server_msg_id else "–")
        self.counters_var.set("gesendet %d · ACK %d · empfangen %d · bestätigt %d"
                              % (self.sent_count, self.ack_count,
                                 self.recv_count, self.delivery_ack_count))
        self.queue_var.set(str(self.net.send_queue.qsize()))
        self.reconnect_var.set(str(self.reconnect_count))
        self.error_var.set(self.last_error)

    def _log(self, text):
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, "[%s] %s\n" % (time.strftime("%H:%M:%S"), text))
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)
        self._refresh_log_scrollbar()

    # -- Beenden --------------------------------------------------------------
    def _on_close(self):
        self.net.stop()
        self.sound.stop()
        self.root.destroy()


def main():
    MorseApp().run()


if __name__ == "__main__":
    main()









