import socket
import threading
import queue
import struct
import select
import time
import http.server
import html
import json
import base64
import os
import re
import subprocess
import sys
from datetime import datetime
from enum import Enum
from collections import OrderedDict, deque

TCP_PORT = 6969
HTTP_PORT = 80   # Status-Endpoint: http://morse-server.de/ (Port < 1024 -> braucht root/CAP_NET_BIND_SERVICE)
JOURNAL_UNIT = "morse-server.service"   # systemd-Unit, deren Journal unter /log angezeigt wird
STATE_FILE = "/var/lib/morse-server/state.json"   # persistenter Zustand (überlebt Neustarts)

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


def save_state():
    """Schreibt registry + msg-id-Zähler atomar in STATE_FILE (überlebt Neustarts)."""
    try:
        with registry_lock:
            data = {
                "next_server_msg_id": next_server_msg_id,
                "devices": {
                    mac.hex(): {
                        "last_seen": entry["last_seen"],
                        "buffer": [
                            {"id": mid, "payload": base64.b64encode(payload).decode("ascii")}
                            for mid, payload in entry["buffer"]
                        ],
                        "recent": [[cid, sid] for cid, sid in entry["recent"].items()],
                    }
                    for mac, entry in registry.items()
                },
            }
        os.makedirs(os.path.dirname(STATE_FILE), exist_ok=True)
        tmp = STATE_FILE + ".tmp"
        with open(tmp, "w") as f:
            json.dump(data, f)
        os.replace(tmp, STATE_FILE)
    except Exception as e:
        log(f"State speichern fehlgeschlagen: ({type(e).__name__}) {e}", ERROR)


def load_state():
    """Lädt registry + msg-id-Zähler aus STATE_FILE (beim Start, vor den Threads)."""
    global next_server_msg_id
    try:
        with open(STATE_FILE) as f:
            data = json.load(f)
    except FileNotFoundError:
        return  # erster Start / keine Datei
    except Exception as e:
        log(f"State laden fehlgeschlagen: ({type(e).__name__}) {e}", ERROR)
        return

    next_server_msg_id = int(data.get("next_server_msg_id", 0))
    for mac_hex, d in data.get("devices", {}).items():
        try:
            mac = bytes.fromhex(mac_hex)
        except ValueError:
            continue
        buffer = deque(
            (int(item["id"]), base64.b64decode(item["payload"]))
            for item in d.get("buffer", [])
        )
        recent = OrderedDict((int(cid), int(sid)) for cid, sid in d.get("recent", []))
        registry[mac] = {
            "handler": None,
            "last_seen": float(d.get("last_seen", time.time())),
            "buffer": buffer,
            "recent": recent,
        }


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


def build_status():
    """Baut eine Live-Übersicht aus dem Speicherzustand (registry + clients).

    Liest NICHT die Logs; die Daten kommen direkt aus den Datenstrukturen,
    die der Server ohnehin laufend pflegt.
    """
    now = time.time()

    with clients_lock:
        unregistered = [
            {"addr": f"{c.client_address[0]}:{c.client_address[1]}"}
            for c in clients
            if c.mac is None
        ]

    with registry_lock:
        devices = []
        for mac, entry in registry.items():
            handler = entry["handler"]
            online = handler is not None
            addr = None
            if online:
                addr = f"{handler.client_address[0]}:{handler.client_address[1]}"
            devices.append({
                "mac": fmt_mac(mac),
                "online": online,
                "addr": addr,
                "last_seen": datetime.fromtimestamp(entry["last_seen"]).strftime("%Y-%m-%d %H:%M:%S"),
                "idle_seconds": int(now - entry["last_seen"]),
                "buffered": len(entry["buffer"]),
            })

    devices.sort(key=lambda d: d["mac"])

    return {
        "now": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "online_count": sum(1 for d in devices if d["online"]),
        "total_known": len(devices),
        "unregistered_connections": unregistered,
        "server_msg_id": next_server_msg_id,
        "devices": devices,
    }


ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")


def strip_ansi(text):
    """Entfernt ANSI-Farbcodes aus einer Log-Zeile."""
    return ANSI_RE.sub("", text)


def _page_head(title, refresh=None):
    refresh_meta = f'<meta http-equiv="refresh" content="{refresh}">' if refresh else ''
    return (
        "<!DOCTYPE html><html><head><meta charset=\"utf-8\">"
        "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
        f"{refresh_meta}"
        f"<title>{title}</title>"
        "<style>"
        "body{font-family:system-ui,sans-serif;background:#111;color:#eee;margin:0;padding:16px}"
        "h1{font-size:20px;margin:0 0 4px}"
        "nav{margin:8px 0 16px}"
        "nav a{color:#4af;text-decoration:none;margin-right:14px}"
        "nav a.active{font-weight:bold;text-decoration:underline}"
        "table{border-collapse:collapse;width:100%;margin-top:12px}"
        "th,td{border:1px solid #333;padding:6px 10px;text-align:left;font-size:14px}"
        ".online{color:#2a7}"
        ".offline{color:#888}"
        "pre{background:#000;padding:12px;border-radius:6px;font-size:12px;"
        "line-height:1.5;overflow-x:auto;white-space:pre-wrap;max-height:75vh;overflow-y:auto}"
        "</style></head><body>"
    )


def _nav(active):
    return (
        "<nav>"
        f"<a href=\"/status\" class=\"{'active' if active == 'status' else ''}\">Status</a>"
        f"<a href=\"/log\" class=\"{'active' if active == 'log' else ''}\">Log</a>"
        "</nav>"
    )


def _read_journal():
    """Liest das Journal der Unit (neueste zuerst) als Liste bereinigter Zeilen."""
    try:
        proc = subprocess.run(
            ["journalctl", "-u", JOURNAL_UNIT, "--no-pager", "--output=cat", "-r"],
            capture_output=True, text=True, timeout=15,
        )
    except (subprocess.TimeoutExpired, OSError) as e:
        return [f"Fehler beim Lesen des Journals: ({type(e).__name__}) {e}"]
    if proc.returncode != 0:
        err = proc.stderr.strip() or f"rc={proc.returncode}"
        return [f"journalctl Fehler: {err}"]
    return [strip_ansi(line) for line in proc.stdout.splitlines()]


def _device_rows_html(devices):
    """Rendert die Tabellenzeilen aller Geräte."""
    rows = []
    for d in devices:
        cls = "online" if d["online"] else "offline"
        label = "online" if d["online"] else "offline"
        rows.append(
            f"<tr class=\"{cls}\"><td>{d['mac']}</td><td>{label}</td>"
            f"<td>{d['addr'] or '–'}</td><td>{d['last_seen']}</td>"
            f"<td>{d['idle_seconds']} s</td><td>{d['buffered']}</td></tr>"
        )
    return "\n".join(rows)


def render_html(status):
    """Baut die HTML-Übersicht (Status-Seite) für den Browser.

    Die Seite lädt alle 1 s automatisch die JSON-Daten (/status.json) nach und
    aktualisiert Tabelle + Zusammenfassung per JavaScript, ohne neu zu laden.
    Ohne JavaScript bleibt die initial gerenderte Ansicht sichtbar.
    """
    parts = [_page_head("Morse-Server Status")]
    parts.append("<h1>Morse-Server Status</h1>")
    parts.append(_nav("status"))
    parts.append(
        f"<p id=\"summary\">{status['online_count']} von {status['total_known']} "
        f"Geräten online (Stand: {status['now']})</p>"
    )
    parts.append(
        "<table><thead><tr><th>MAC</th><th>Status</th><th>Adresse</th>"
        "<th>Letzter Kontakt</th><th>Idle</th><th>Puffer</th></tr></thead>"
        "<tbody id=\"devices\">"
    )
    parts.append(_device_rows_html(status["devices"]))
    parts.append("</tbody></table>")

    unreg = ""
    if status["unregistered_connections"]:
        unreg = "Verbindungen ohne Registrierung: " + ", ".join(
            c["addr"] for c in status["unregistered_connections"]
        )
    parts.append(f"<p id=\"unregistered\">{html.escape(unreg)}</p>")

    parts.append(
        """<script>
async function refresh(){
  try{
    const r=await fetch('/status.json');const s=await r.json();
    document.getElementById('summary').textContent=
      s.online_count+' von '+s.total_known+' Geräten online (Stand: '+s.now+')';
    document.getElementById('devices').innerHTML=s.devices.map(function(d){
      var cls=d.online?'online':'offline';
      var label=d.online?'online':'offline';
      return '<tr class="'+cls+'"><td>'+d.mac+'</td><td>'+label+'</td>'+
             '<td>'+(d.addr||'–')+'</td><td>'+d.last_seen+'</td>'+
             '<td>'+d.idle_seconds+' s</td><td>'+d.buffered+'</td></tr>';
    }).join('');
    var u=document.getElementById('unregistered');
    if(s.unregistered_connections&&s.unregistered_connections.length){
      u.textContent='Verbindungen ohne Registrierung: '+
        s.unregistered_connections.map(function(c){return c.addr;}).join(', ');
    }else{u.textContent='';}
  }catch(e){}
}
setInterval(refresh,1000);
</script>"""
    )
    parts.append("</body></html>")
    return "\n".join(parts)


def render_log_html():
    """Baut die Log-Seite (alle Journal-Zeilen, neueste zuerst) für den Browser."""
    lines = _read_journal()
    parts = [_page_head("Morse-Server Log")]
    parts.append("<h1>Morse-Server Log</h1>")
    parts.append(_nav("log"))
    parts.append(f"<p>{len(lines)} Zeilen (neueste zuerst)</p>")
    parts.append("<pre>" + "\n".join(html.escape(line) for line in lines) + "</pre>")
    parts.append("</body></html>")
    return "\n".join(parts)


class MorseStatusHTTPServer(http.server.ThreadingHTTPServer):
    """ThreadingHTTPServer, das Verbindungsabbrüche ohne Traceback loggt."""

    def handle_error(self, request, client_address):
        exc_type, _, _ = sys.exc_info()
        # Port-Scanner/Bots und abgebrochene Browser-Requests verursachen
        # ConnectionReset/Abort/BrokenPipe – das ist harmloser Log-Spam.
        if exc_type is not None and issubclass(exc_type, (ConnectionError, TimeoutError)):
            log(f"HTTP-Client abgebrochen {client_address}: ({exc_type.__name__})", WARNING)
        else:
            super().handle_error(request, client_address)


class StatusHandler(http.server.BaseHTTPRequestHandler):
    """Liefert Status (/status), Log (/log) und leitet / auf /status weiter."""

    def _respond(self, code, content_type, body):
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _redirect(self, location):
        self.send_response(302)
        self.send_header("Location", location)
        self.send_header("Content-Length", "0")
        self.end_headers()

    def _build_status(self):
        try:
            return build_status(), None
        except Exception as e:
            return None, f"({type(e).__name__}): {e}"

    def do_GET(self):
        path = self.path.split("?", 1)[0]

        # Startseite -> Status-Seite
        if path in ("/", "/index.html"):
            self._redirect("/status")
            return

        if path == "/status":
            status, err = self._build_status()
            if err:
                log(f"Status-Endpoint Fehler: {err}", ERROR)
                self._respond(500, "text/plain; charset=utf-8", b"Internal Server Error")
                return
            self._respond(200, "text/html; charset=utf-8", render_html(status).encode("utf-8"))
            return

        if path == "/status.json":
            status, err = self._build_status()
            if err:
                log(f"Status-Endpoint Fehler: {err}", ERROR)
                self._respond(500, "text/plain; charset=utf-8", b"Internal Server Error")
                return
            self._respond(200, "application/json; charset=utf-8", json.dumps(status, indent=2).encode("utf-8"))
            return

        if path == "/log":
            self._respond(200, "text/html; charset=utf-8", render_log_html().encode("utf-8"))
            return

        self._respond(404, "text/plain; charset=utf-8", b"Not Found")

    def log_message(self, fmt, *args):
        # Zugriffe landen im Server-Log (journal) statt auf stderr.
        # Bots/Scanner probieren wilde Pfade aus -> 404er sind Log-Rauschen.
        if len(args) >= 2 and str(args[1]) == "404":
            return
        # Das 1-s-Polling von /status.json (Live-Update) ist ebenfalls nur Rauschen.
        if getattr(self, "path", "").split("?", 1)[0] == "/status.json":
            return
        log("HTTP %s" % (fmt % args), INFO)


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
                    save_state()


def main():
    running = True
    httpd = None

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", TCP_PORT))
    server.listen()
    server.settimeout(1.0)      # wichtig für kontrolliertes Schließen

    load_state()   # persistente Clients + Puffer + msg-id-Zähler wiederherstellen

    reaper = threading.Thread(target=reaper_loop, daemon=True)
    reaper.start()

    log("Server startet...", GOOD_INFO)
    log_table_header()

    # --- HTTP-Status-Endpoint (Live-Übersicht, Port 80) ---
    try:
        httpd = MorseStatusHTTPServer(("0.0.0.0", HTTP_PORT), StatusHandler)
        threading.Thread(target=httpd.serve_forever, daemon=True).start()
        log(f"HTTP-Status-Endpoint auf 0.0.0.0:{HTTP_PORT} (/)", GOOD_INFO)
    except PermissionError:
        log(f"HTTP-Status-Endpoint: Port {HTTP_PORT} braucht root oder CAP_NET_BIND_SERVICE", ERROR)
    except OSError as e:
        log(f"HTTP-Status-Endpoint konnte nicht starten: {e}", ERROR)

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
        save_state()   # letzten Stand sichern
        running = False
        server.close()
        if httpd is not None:
            httpd.shutdown()
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
        # CONNECT wird nicht für jede rohe Verbindung geloggt: Bots/Scanner
        # hämmern auf Port 6969. Ein echtes Gerät meldet sich über REGISTER.
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
        save_state()

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
        save_state()

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
        save_state()

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
        save_state()

    @staticmethod
    def _trim_recent(entry):
        while len(entry["recent"]) > RECENT_MAX:
            entry["recent"].popitem(last=False)

    def _on_disconnect(self, reason="unknown"):
        # Nur für registrierte Geräte loggen (Bots/Scanner -> mac ist None).
        if self.mac is not None:
            log_event("----", "DISCONNECT", mac=self.mac,
                      note=f"addr={self.client_address} reason={reason}")
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
