// =============================================================================
// network.cpp  –  WLAN, DNS und TCP (niedrige Netzwerk-Ebene)
//
// Enthält die eigentlichen ESP-IDF-/lwIP-Aufrufe:
//   - WLAN initialisieren und verbinden (inkl. IPv6-Adresse abwarten)
//   - Hostname per DNS auflösen
//   - TCP-Socket öffnen, verbinden, prüfen
//   - Pakete senden/empfangen (Header + Payload)
//
// Das Netzwerk-Protokoll eines Pakets (muss zum Server passen):
//   Byte 0    : status (0=keepalive, 1=normal, 2=Bestätigung, 3=server_check)
//   Byte 1..2 : Größe des Payloads (16 Bit, Big-Endian)
//   Byte 3..  : Payload (die Morse-Bytes)
// =============================================================================

#include "network.h"

#include <esp_event.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <esp_wifi.h>

#include <cstdio>
#include <string.h>

#include <errno.h>
#include <fcntl.h>
#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include <lwip/inet.h>

#include "app_state.h"
#include "config.h"
#include "package.h"

static const char* TAG = "network";
static esp_netif_t* sta_netif = nullptr;

// Wird vom WLAN-Event-Handler gesetzt, wenn ein Verbindungsversuch "endgültig"
// gescheitert ist (AP nicht gefunden, falsches Passwort, ...). connectWifi()
// bricht dann sein Warten frühzeitig ab, statt die vollen ~10 s zu warten.
// `volatile`, weil der Event-Handler in einem anderen Task läuft.
static volatile bool wifi_connect_aborted = false;

// Timeouts der Netzwerk-Schicht.
static constexpr int POLL_TIMEOUT_MS = 100;    // select() wartet max. so lange auf Daten
static constexpr int CONNECT_TIMEOUT_S = 5;    // connect() wartet max. so lange
static constexpr int SOCKET_IO_TIMEOUT_S = 5;  // recv()/send() blockieren max. so lange

static const char* wifiEventName(int32_t id) {
  switch (id) {
    case WIFI_EVENT_STA_START: return "STA_START";
    case WIFI_EVENT_STA_CONNECTED: return "STA_CONNECTED";
    case WIFI_EVENT_STA_DISCONNECTED: return "STA_DISCONNECTED";
    case WIFI_EVENT_STA_STOP: return "STA_STOP";
    case WIFI_EVENT_SCAN_DONE: return "SCAN_DONE";
    case WIFI_EVENT_STA_AUTHMODE_CHANGE: return "STA_AUTHMODE_CHANGE";
    case WIFI_EVENT_STA_BEACON_TIMEOUT: return "STA_BEACON_TIMEOUT";
    case WIFI_EVENT_HOME_CHANNEL_CHANGE: return "HOME_CHANNEL_CHANGE";
    default: return "UNKNOWN";
  }
}

static const char* ipEventName(int32_t id) {
  switch (id) {
    case IP_EVENT_STA_GOT_IP: return "STA_GOT_IP";
    case IP_EVENT_STA_LOST_IP: return "STA_LOST_IP";
    case IP_EVENT_GOT_IP6: return "GOT_IP6";
    case IP_EVENT_NETIF_UP: return "NETIF_UP";
    case IP_EVENT_NETIF_DOWN: return "NETIF_DOWN";
    default: return "UNKNOWN";
  }
}

static void wifiEventHandler(void* arg, esp_event_base_t base, int32_t id, void* data) {
  if (id == WIFI_EVENT_STA_DISCONNECTED) {
    auto* d = static_cast<wifi_event_sta_disconnected_t*>(data);
    ESP_LOGW(TAG, "Disconnected: %s (%d)", disconnectReason(d->reason), d->reason);

    // "Endgültige" Fehlergründe: Da hilft Weiterwarten auf dieselbe SSID
    // nichts (AP nicht da bzw. Passwort falsch). Das Flag lässt connectWifi()
    // sofort aufgeben, damit schnell das Fallback-Netz probiert wird.
    switch (d->reason) {
      case WIFI_REASON_NO_AP_FOUND:         // AP nach vollem Scan nicht gefunden
      case WIFI_REASON_AUTH_FAIL:           // Passwort falsch / abgelehnt
      case WIFI_REASON_ASSOC_FAIL:          // Assoziierung abgelehnt
      case WIFI_REASON_HANDSHAKE_TIMEOUT:   // WPA-Handshake lief in Timeout
      case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
        wifi_connect_aborted = true;
        break;
      default:
        break;  // harmlos (z. B. eigener esp_wifi_disconnect()) -> ignorieren
    }
  } else {
    ESP_LOGI(TAG, "WiFi event: %s (%ld)", wifiEventName(id), (long)id);
  }
}

static void ipEventHandler(void* arg, esp_event_base_t base, int32_t id, void* data) {
  if (id == IP_EVENT_STA_GOT_IP) {
    auto* d = static_cast<ip_event_got_ip_t*>(data);
    ESP_LOGI(TAG, "IPv4: %s", inet_ntoa(d->ip_info.ip));  // IPv4-Adresse (Binär) -> Text
  } else if (id == IP_EVENT_GOT_IP6) {
    auto* d = static_cast<ip_event_got_ip6_t*>(data);
    char addrStr[INET6_ADDRSTRLEN];
    inet_ntop(AF_INET6, d->ip6_info.ip.addr, addrStr, sizeof(addrStr));  // IPv6 -> Text
    ESP_LOGI(TAG, "IPv6: %s", addrStr);
  } else {
    ESP_LOGI(TAG, "IP event: %s (%ld)", ipEventName(id), (long)id);
  }
}

// ---------------------------------------------------------------------
// WLAN
// ---------------------------------------------------------------------

void wifiInit() {
  // Standard-WLAN-Schnittstelle "Station" (= Client) anlegen und den Zeiger
  // für später merken (wir brauchen ihn für die IPv6-Adressen).
  sta_netif = esp_netif_create_default_wifi_sta();

  // WLAN-Treiber initialisieren. WIFI_INIT_CONFIG_DEFAULT() liefert eine
  // fertige Standard-Konfiguration (Event-Task, Puffergrößen usw.).
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  // WLAN-Einstellungen NICHT in den Flash (NVS) schreiben – RAM genügt hier.
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

  // Stromsparmodus ausschalten -> geringere Latenz (kein Aufwachen nötig).
  esp_wifi_set_ps(WIFI_PS_NONE);

  // Event-Handler registrieren: Unsere Funktionen werden bei WLAN- und
  // IP-Ereignissen automatisch aufgerufen (nur für Log-Ausgaben).
  // ESP_EVENT_ANY_ID = auf ALLE Events dieser Gruppe reagieren.
  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifiEventHandler, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, &ipEventHandler, NULL));

  ESP_LOGI(TAG, "WLAN initialisiert");
}

// Wartet, bis eine globale IPv6-Adresse (und damit eine Route zum Server)
// vorhanden ist. Die globale Adresse wird per Router Advertisement (SLAAC)
// vergeben, und der Router schickt RAs teils nur alle paar Sekunden. Deshalb
// wird hier großzügig gewartet. Muss nach jeder WLAN-(Re-)Verbindung aufgerufen
// werden, sonst schlägt der IPv6-TCP-Verbindungsaufbau mit "Host is unreachable"
// fehl (Fehler 118 = ENETUNREACH).
bool waitForIPv6Address() {
  // Link-Local-Adresse anlegen (nötig für SLAAC; erst möglich, wenn das
  // Interface "up" ist). Nach einem Reconnect ggf. erneut nötig. Der Aufruf
  // ist idempotent (existiert die Adresse schon, passiert nichts).
  bool linklocal_ok = false;
  for (int i = 0; i < 50; i++) {
    // WLAN inzwischen abgebrochen? -> sofort aufgeben, damit die
    // Zustandsmaschine nicht sinnlos ~5 s wartet.
    if (!wifiIsConnected())
      return false;
    if (esp_netif_create_ip6_linklocal(sta_netif) == ESP_OK) {
      linklocal_ok = true;
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  ESP_LOGI(TAG, "IPv6 linklocal: %s", linklocal_ok ? "OK" : "FAILED");

  // Auf die globale IPv6-Adresse warten (bis zu ~20 s).
  for (int i = 0; i < 40; i++) {
    // WLAN abgebrochen? -> sofort aufgeben (sonst 20 s sinnlos warten).
    if (!wifiIsConnected())
      return false;
    esp_ip6_addr_t addr;
    // Globale IPv6-Adresse abfragen. Liefert ESP_OK, sobald der Router uns
    // per Router Advertisement eine Adresse zugewiesen hat.
    if (esp_netif_get_ip6_global(sta_netif, &addr) == ESP_OK) {
      char addrStr[INET6_ADDRSTRLEN];
      inet_ntop(AF_INET6, addr.addr, addrStr, sizeof(addrStr));  // Binär -> Text
      ESP_LOGI(TAG, "IPv6 global: %s", addrStr);
      return true;
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }

  ESP_LOGW(TAG, "Didn't get IPv6 address");
  return false;
}

bool connectWifi(const char ssid[], const char password[]) {
  // Alten TCP-Socket (falls vorhanden) schließen und zurück in den WLAN-Zustand.
  disconnectTCP();
  state = WIFI_CONNECT;

  // WLAN-Treiber sauber neu aufsetzen: erst Verbindung trennen, dann stoppen.
  esp_wifi_disconnect();
  esp_wifi_stop();

  // Betriebsmodus festlegen: WIFI_MODE_STA = Station (Client, kein Access Point).
  if (esp_wifi_set_mode(WIFI_MODE_STA) != ESP_OK) {
    ESP_LOGE(TAG, "set_mode STA failed");
    return false;
  }

  // SSID (Netzwerkname) und Passwort in die Konfiguration kopieren.
  wifi_config_t wifi_config = {};
  strlcpy(reinterpret_cast<char*>(wifi_config.sta.ssid), ssid, sizeof(wifi_config.sta.ssid));
  strlcpy(reinterpret_cast<char*>(wifi_config.sta.password), password, sizeof(wifi_config.sta.password));

  // Die Konfiguration (SSID/Passwort) an den WLAN-Treiber übergeben.
  if (esp_wifi_set_config(WIFI_IF_STA, &wifi_config) != ESP_OK) {
    ESP_LOGE(TAG, "set_config failed");
    return false;
  }

  // WLAN-Treiber (wieder) starten.
  if (esp_wifi_start() != ESP_OK) {
    ESP_LOGE(TAG, "wifi start failed");
    return false;
  }

  ESP_LOGI(TAG, "Connecting to %s", ssid);

  // Flag zurücksetzen: Ab JETZT zählt ein endgültiger Fehler als Abbruch-Grund.
  // (Der Disconnect, den esp_wifi_disconnect()/esp_wifi_stop() weiter oben
  // auslösen, hat einen harmlosen Grund und setzt das Flag nicht.)
  wifi_connect_aborted = false;

  // Verbindungsaufbau zum konfigurierten Access Point anstoßen.
  esp_err_t err = esp_wifi_connect();
  if (err != ESP_OK)
    ESP_LOGW(TAG, "connect: %s", esp_err_to_name(err));  // Fehlercode als Text ausgeben

  // Auf Verbindung warten (max. ~10 s), aber SOFORT abbrechen, sobald der
  // Treiber einen endgültigen Fehler meldet (AP nicht gefunden, falsches
  // Passwort, ...). Das spart bei fehlenden Netzen mehrere Sekunden.
  for (int i = 0; i < 200 && !wifi_connect_aborted; i++) {
    if (wifiIsConnected())
      break;
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  if (!wifiIsConnected()) {
    ESP_LOGE(TAG, "Couldn't connect to %s", ssid);
    return false;
  }

  ESP_LOGI(TAG, "WiFi OK");
  state = WAIT_FOR_IP6;
  // Auf die globale IPv6-Adresse warten (siehe waitForIPv6Address()).
  return waitForIPv6Address();
}

bool wifiIsConnected() {
  wifi_ap_record_t ap;
  // Infos des aktuell verbundenen Access Points abfragen.
  // Liefert ESP_OK, wenn eine Verbindung steht -> dann sind wir verbunden.
  return esp_wifi_sta_get_ap_info(&ap) == ESP_OK;
}

int wifiRssi() {
  wifi_ap_record_t ap;
  // Signalstärke (RSSI in dBm) aus den AP-Infos lesen.
  if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK)
    return ap.rssi;
  return -127;  // -127 = kein Signal / nicht verbunden
}

const char* disconnectReason(uint8_t reason) {
  switch (reason) {
    case WIFI_REASON_UNSPECIFIED: return "UNSPECIFIED";
    case WIFI_REASON_AUTH_EXPIRE: return "AUTH_EXPIRE";
    case WIFI_REASON_AUTH_LEAVE: return "AUTH_LEAVE";
    case WIFI_REASON_DISASSOC_DUE_TO_INACTIVITY: return "DISASSOC_DUE_TO_INACTIVITY";
    case WIFI_REASON_ASSOC_TOOMANY: return "ASSOC_TOOMANY";
    case WIFI_REASON_CLASS2_FRAME_FROM_NONAUTH_STA: return "CLASS2_FRAME_FROM_NONAUTH_STA";
    case WIFI_REASON_CLASS3_FRAME_FROM_NONASSOC_STA: return "CLASS3_FRAME_FROM_NONASSOC_STA";
    case WIFI_REASON_ASSOC_LEAVE: return "ASSOC_LEAVE";
    case WIFI_REASON_ASSOC_NOT_AUTHED: return "ASSOC_NOT_AUTHED";
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT: return "4WAY_HANDSHAKE_TIMEOUT";
    case WIFI_REASON_HANDSHAKE_TIMEOUT: return "HANDSHAKE_TIMEOUT";
    case WIFI_REASON_NO_AP_FOUND: return "NO_AP_FOUND";
    case WIFI_REASON_AUTH_FAIL: return "AUTH_FAIL";
    case WIFI_REASON_ASSOC_FAIL: return "ASSOC_FAIL";
    default: return "UNKNOWN";
  }
}

// ---------------------------------------------------------------------
// DNS + TCP
// ---------------------------------------------------------------------

void disconnectTCP() {
  // Socket schließen, falls er offen ist.
  if (sock >= 0)
    close(sock);
  sock = -1;  // markieren: kein Socket offen
}

bool resolveDNS() {
  struct addrinfo hints = {};
  struct addrinfo* result = nullptr;

  // Hinweise für die DNS-Auflösung: wir wollen IPv6 (AF_INET6) + TCP.
  hints.ai_family = AF_INET6;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_protocol = IPPROTO_TCP;

  // Portnummer in einen String umwandeln (getaddrinfo erwartet Text).
  char portString[8];
  snprintf(portString, sizeof(portString), "%d", port);

  // Hostname + Port in eine IP-Adresse auflösen (DNS-Anfrage).
  int err = getaddrinfo(server_address, portString, &hints, &result);
  if (err != 0 || result == nullptr) {
    ESP_LOGE(TAG, "IPv6 DNS failed, error=%d", err);
    return false;
  }

  // Alle gefundenen Adressen ausgeben (Debug).
  int i = 0;
  for (struct addrinfo* p = result; p != nullptr; p = p->ai_next) {
    char addrStr[INET6_ADDRSTRLEN] = {};
    struct sockaddr_in6* addr6 = reinterpret_cast<struct sockaddr_in6*>(p->ai_addr);
    inet_ntop(AF_INET6, &(addr6->sin6_addr), addrStr, sizeof(addrStr));  // Binär -> Text
    ESP_LOGI(TAG, "DNS result %d: [%s]:%d", i++, addrStr, ntohs(addr6->sin6_port));  // ntohs: Port in lesbare Reihenfolge
  }

  // Erste Adresse übernehmen (für den späteren TCP-connect).
  memcpy(&server_addr, result->ai_addr, result->ai_addrlen);
  server_addr_len = result->ai_addrlen;

  ESP_LOGI(TAG, "IPv6 DNS OK");
  freeaddrinfo(result);  // vom DNS belegten Speicher wieder freigeben
  return true;
}

bool connectTCP(int retry_counter) {
  ESP_LOGI(TAG, "Connecting TCP...");

  // TCP-Socket öffnen: IPv6 (AF_INET6), verbindungsorientiert (SOCK_STREAM).
  sock = socket(AF_INET6, SOCK_STREAM, IPPROTO_TCP);
  if (sock < 0) {
    ESP_LOGE(TAG, "IPv6 socket failed: %s", strerror(errno));  // strerror: Fehlercode -> Text
    return false;
  }

  // Socket nicht-blockierend schalten, damit connect() nicht endlos blockiert
  // (lwip wartet sonst ~26 s auf eine SYN-Antwort). Wir warten stattdessen
  // unten mit select() auf den Verbindungsabschluss.
  int flags = fcntl(sock, F_GETFL, 0);
  fcntl(sock, F_SETFL, flags | O_NONBLOCK);

  int ret = connect(sock, reinterpret_cast<struct sockaddr*>(&server_addr), server_addr_len);
  if (ret != 0 && errno != EINPROGRESS) {
    // Sofortiger Fehler (z. B. keine Route -> "Host is unreachable").
    ESP_LOGE(TAG, "IPv6 connect failed: %s (%d)", strerror(errno), errno);
    close(sock);
    sock = -1;
    return false;
  }

  // Max. CONNECT_TIMEOUT_S auf den Verbindungsabschluss warten: Der Socket wird
  // "schreibbar", sobald die Verbindung steht (oder endgültig fehlgeschlagen ist).
  fd_set wfds;
  FD_ZERO(&wfds);
  FD_SET(sock, &wfds);
  timeval tv{ CONNECT_TIMEOUT_S, 0 };
  int sel = select(sock + 1, nullptr, &wfds, nullptr, &tv);

  if (sel <= 0) {
    ESP_LOGE(TAG, "IPv6 connect timeout");
    close(sock);
    sock = -1;
    return false;
  }

  // Prüfen, ob die Verbindung wirklich zustande kam (SO_ERROR muss 0 sein).
  int so_error = 0;
  socklen_t so_len = sizeof(so_error);
  getsockopt(sock, SOL_SOCKET, SO_ERROR, &so_error, &so_len);
  if (so_error != 0) {
    ESP_LOGE(TAG, "IPv6 connect failed: %s (%d)", strerror(so_error), so_error);
    close(sock);
    sock = -1;
    return false;
  }

  // Socket wieder blockierend schalten (für die normale Nutzung).
  fcntl(sock, F_SETFL, flags);

  ESP_LOGI(TAG, "IPv6 TCP connected");

  // --- TCP-Keepalive: erkennt still abgerissene Verbindungen. ---
  // setsockopt() setzt eine Socket-Option (hier: Keepalive einschalten).
  int yes = 1;
  setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &yes, sizeof(yes));
  int idle = 10;                                       // nach 10 s Inaktivität prüfen
  setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
  int interval = 5;                                    // alle 5 s erneut prüfen
  setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &interval, sizeof(interval));
  int count = 3;                                       // nach 3 Fehlversuchen aufgeben
  setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &count, sizeof(count));

  // --- Sende-/Empfangs-Timeout: recv/send blockieren max. SOCKET_IO_TIMEOUT_S. ---
  timeval timeout;
  timeout.tv_sec = SOCKET_IO_TIMEOUT_S;
  timeout.tv_usec = 0;
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

  state = RUNNING;
  return true;
}

bool checkSocket(bool& readable) {
  // 1) Socket-Fehlerstatus abfragen (z. B. "Connection reset by peer").
  int err;
  socklen_t len = sizeof(err);
  getsockopt(sock, SOL_SOCKET, SO_ERROR, &err, &len);  // liest eine Socket-Option

  if (err) {
    ESP_LOGE(TAG, "Socket error: %s (%d)", strerror(err), err);
    return false;
  }

  if (sock < 0) {
    ESP_LOGW(TAG, "socket disconnected returning to DNS_RESOLVE");
    return false;
  }

  readable = false;

  fd_set readfds;          // Menge von Dateideskriptoren ("FD-Set")
  FD_ZERO(&readfds);       // Menge leeren
  FD_SET(sock, &readfds);  // unseren Socket in die Menge aufnehmen

  // 2) select() wartet (blockierend) auf eingehende Daten. WICHTIG: nur auf
  //    "lesbar" warten, NICHT auf "schreibbar" – ein verbundener Socket ist
  //    fast immer schreibbar, wodurch select sonst SOFORT zurückkehren und die
  //    Schleife zur Busy-Loop werden würde (-> Task-Watchdog auf CPU 0).
  //    Mit dem POLL_TIMEOUT_MS-Timeout gibt select die CPU an den IDLE-Task ab.
  timeval tv{ 0, POLL_TIMEOUT_MS * 1000 };  // 0 s + 100 ms

  int ret = select(sock + 1, &readfds, nullptr, nullptr, &tv);

  if (ret < 0) {
    ESP_LOGE(TAG, "select failed: %d", errno);
    return false;
  }

  readable = FD_ISSET(sock, &readfds);  // ist der Socket in der Menge (also lesbar)?

  return true;
}

// ---------------------------------------------------------------------
// Senden / Empfangen
// ---------------------------------------------------------------------

bool sendAll(const uint8_t* data, size_t len) {
  size_t sent = 0;

  // Solange senden, bis wirklich ALLE Bytes geschickt wurden (send() kann
  // weniger schicken als angefordert).
  while (sent < len) {
    int n = send(sock, data + sent, len - sent, 0);  // Daten über den Socket schicken
    if (n > 0) {
      sent += n;  // ein Teil wurde geschickt -> weitermachen
    } else if (n == 0) {
      ESP_LOGW(TAG, "peer disconnected");
      disconnectTCP();
      state = TCP_CONNECT;
      return false;
    } else {
      ESP_LOGE(TAG, "send failed: %d", errno);  // errno = letzter Fehlercode
      disconnectTCP();
      state = TCP_CONNECT;
      return false;
    }
  }
  return true;
}

bool recvAll(uint8_t* bytes, size_t bytesToRead) {
  size_t got = 0;

  // Solange empfangen, bis exakt die gewünschte Byte-Anzahl angekommen ist.
  while (got < bytesToRead) {
    // MSG_WAITALL = warten, bis der Puffer voll ist (bzw. Fehler/Timeout).
    int n = recv(sock, bytes + got, bytesToRead - got, MSG_WAITALL);

    if (n > 0) {
      got += n;  // ein Teil ist angekommen -> weitermachen
    } else if (n == 0) {
      ESP_LOGW(TAG, "peer disconnected");  // 0 = Gegenseite hat sauber geschlossen
      disconnectTCP();
      state = TCP_CONNECT;
      return false;
    } else {
      ESP_LOGE(TAG, "recv failed: %d", errno);  // errno = letzter Fehlercode
      disconnectTCP();
      state = TCP_CONNECT;
      return false;
    }
  }

  return true;
}

// Feste Bestätigung ("Paket empfangen", status 2) an den Server schicken.
// Das Bitmuster ist ein vereinbartes Erkennungssignal.
static void sendConfirmation() {
  Package confirmation;
  confirmation.status = 2;
  confirmation.payload = { 0b11111111, 0, 0b11111111, 0b11111111, 0b11111111, 0, 0b11111111 };
  confirmation.size = confirmation.payload.size();

  if (!putPackageIntoQueue(sendQueue, confirmation))
    ESP_LOGW(TAG, "sendQueue overflow");
}

void receivePackage() {
  Package incoming;

  // Ein Paket beginnt immer mit einem 3-Byte-Header (status + size).
  uint8_t header[3];
  if (!recvAll(header, 3))
    return;

  incoming.status = header[0];
  incoming.size = (uint16_t(header[1]) << 8) | header[2];  // Big-Endian

  // Dann den Payload in der im Header angegebenen Größe lesen.
  incoming.payload.resize(incoming.size);
  if (!recvAll(incoming.payload.data(), incoming.size))
    return;

  ESP_LOGI(TAG, "%s", packageToText(incoming).c_str());

  // Empfangenes Paket an die Wiedergabe (playbackQueue) weiterreichen.
  if (!putPackageIntoQueue(playbackQueue, incoming))
    ESP_LOGW(TAG, "playbackQueue overflow");

  // Bestätigungen (status 2) und Server-Check-Antworten (status 3) brauchen
  // keine weitere Bestätigung -> hier abbrechen.
  if (incoming.status == 2)
    return;
  if (incoming.status == 3)
    return;

  // Normales Paket (status 1): dem Server eine feste Bestätigung zurücksenden.
  sendConfirmation();
}

void sendPackage() {
  Package outgoing;
  // Gibt es überhaupt ein zu sendendes Paket?
  if (uxQueueMessagesWaiting(sendQueue) == 0)
    return;
  getPackageFromQueue(sendQueue, outgoing);

  // Komplettes Paket zusammensetzen: Header (3 Bytes) + Payload.
  std::vector<uint8_t> packet;
  packet.push_back(outgoing.status);                 // Byte 0: status
  packet.push_back((outgoing.size >> 8) & 0xFF);     // Byte 1: size (High)
  packet.push_back(outgoing.size & 0xFF);            // Byte 2: size (Low)
  packet.insert(packet.end(), outgoing.payload.begin(), outgoing.payload.end());

  // Paket senden (blockiert max. SO_SNDTIMEO = 5 s).
  sendAll(packet.data(), packet.size());
}
