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
//   Byte 0     : status (0=keepalive, 1=Nachricht, 2=ACK, 3=check, 4=register)
//   Byte 1..8  : msg_id (64 Bit, Big-Endian; Bedeutung je Richtung)
//   Byte 9..10 : Größe des Payloads (16 Bit, Big-Endian)
//   Byte 11..  : Payload (Morse-Bytes bzw. beim Register die 6-Byte-MAC)
// =============================================================================

#include "network.h"

#include <esp_event.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <esp_wifi.h>
#include <nvs.h>

#include <cstdio>
#include <string.h>

#include <errno.h>
#include <fcntl.h>
#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include <lwip/inet.h>
#include <lwip/dns.h>

#include "app_state.h"
#include "config.h"
#include "package.h"

#include <esp_system.h>   // esp_random
#include <esp_mac.h>      // esp_read_mac
#include <esp_timer.h>    // esp_timer_get_time

#include <algorithm>      // std::find
#include <deque>

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

// ------------------------- Identität & Sendepuffer -------------------------
// Eigene MAC-Adresse (Identität gegenüber dem Server).
static uint8_t device_mac[6] = { 0 };

// Ausstehende, noch nicht bestätigte Sendungen (für Retry bei fehlendem ACK).
struct PendingSend {
  Package pkg;         // enthält status, msg_id (client_msg_id) und payload
  uint32_t last_send_ms;
  int retries;
};
static std::vector<PendingSend> pendingSends;

// Zuletzt verarbeitete server_msg_ids (Dedupe gegen doppeltes Zustellen).
static constexpr size_t PROCESSED_MAX = 32;
static std::deque<uint64_t> processedIds;

static uint32_t nowMs() {
  return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

// Liefert einen lesbaren Text für einen Socket-Fehlercode. Ein Timeout
// (SO_RCVTIMEO/SO_SNDTIMEO abgelaufen) wird explizit als "timeout" markiert,
// damit man ihn im Log klar von "reset"/"unreachable" unterscheiden kann.
static const char* socketErrText(int err) {
  if (err == EAGAIN || err == EWOULDBLOCK || err == ETIMEDOUT)
    return "timeout";
  return strerror(err);
}

// -------------------------------------------------------------------------
// client_msg_id: monotoner Zähler, der im NVS (Flash) überlebt. So gibt es
// nach einem Reboot keine Kollision mit alten Einträgen des Servers.
// -------------------------------------------------------------------------
static constexpr const char* NVS_NAMESPACE = "morse";
static constexpr const char* NVS_KEY_MSG_ID = "client_msg_id";

static uint64_t loadClientMsgId() {
  nvs_handle_t handle;
  uint64_t last = 0;
  if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) == ESP_OK) {
    size_t len = sizeof(last);
    nvs_get_blob(handle, NVS_KEY_MSG_ID, &last, &len);
    nvs_close(handle);
  }
  return last;
}

static void saveClientMsgId(uint64_t last) {
  nvs_handle_t handle;
  if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK) {
    nvs_set_blob(handle, NVS_KEY_MSG_ID, &last, sizeof(last));
    nvs_commit(handle);
    nvs_close(handle);
  }
}

// Eindeutige client_msg_id: monotoner Zähler, der im NVS überlebt.
static uint64_t newClientMsgId() {
  static uint64_t last_used = 0;
  static bool loaded = false;

  if (!loaded) {
    last_used = loadClientMsgId();  // 0, wenn noch nichts gespeichert ist
    loaded = true;
  }

  uint64_t id = last_used + 1;
  last_used = id;
  saveClientMsgId(last_used);
  return id;
}

// Liest die eigene MAC-Adresse (einmal beim Start, in wifiInit aufgerufen).
static void readDeviceMac() {
  if (esp_read_mac(device_mac, ESP_MAC_WIFI_STA) != ESP_OK) {
    memset(device_mac, 0, sizeof(device_mac));
    ESP_LOGE(TAG, "MAC auslesen fehlgeschlagen");
  } else {
    ESP_LOGI(TAG, "MAC: %02x:%02x:%02x:%02x:%02x:%02x",
             device_mac[0], device_mac[1], device_mac[2],
             device_mac[3], device_mac[4], device_mac[5]);
  }
}

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
  } else {
    ESP_LOGI(TAG, "IP event: %s (%ld)", ipEventName(id), (long)id);
  }
}

// ---------------------------------------------------------------------
// WLAN
// ---------------------------------------------------------------------

void wifiInit() {
  // Standard-WLAN-Schnittstelle "Station" (= Client) anlegen und den Zeiger
  // für später merken (wir brauchen ihn für die IPv4-Adresse).
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

  // Eigene MAC-Adresse als Geräte-Identität auslesen.
  readDeviceMac();

  ESP_LOGI(TAG, "WLAN initialisiert");
}

// Wartet, bis per DHCP eine IPv4-Adresse vergeben wurde (und damit eine Route
// zum Server existiert). Muss nach jeder WLAN-(Re-)Verbindung aufgerufen
// werden, sonst schlägt der DNS-/TCP-Verbindungsaufbau fehl. DHCP ist schnell,
// deshalb reicht eine kurze Warteschleife (bis zu ~20 s).
bool waitForIPv4Address() {
  for (int i = 0; i < 40; i++) {
    // WLAN inzwischen abgebrochen? -> sofort aufgeben.
    if (!wifiIsConnected())
      return false;

    esp_netif_ip_info_t info;
    // Liefert ESP_OK, sobald der Router uns per DHCP eine IPv4-Adresse
    // zugewiesen hat (info.ip.addr ist dann ungleich 0).
    if (esp_netif_get_ip_info(sta_netif, &info) == ESP_OK && info.ip.addr != 0) {
      ESP_LOGI(TAG, "IPv4: %s", inet_ntoa(info.ip));  // Binär -> Text
      return true;
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }

  ESP_LOGW(TAG, "Didn't get IPv4 address");
  return false;
}

// Setzt den WLAN-Abbruch-Merker zurück. Wird vor einem Verbindungsversuch
// aufgerufen (sowohl in connectWifi() als auch im Reconnect-Pfad).
void resetWifiAbort() {
  wifi_connect_aborted = false;
}

// true, wenn der WLAN-Treiber einen "endgültigen" Fehler gemeldet hat
// (NO_AP_FOUND, AUTH_FAIL, ...). Erlaubt es, Warteschleifen frühzeitig
// abzubrechen, statt die vollen ~10 s zu warten.
bool wifiAbortRequested() {
  return wifi_connect_aborted;
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
  // Auf die IPv4-Adresse warten (DHCP, siehe waitForIPv4Address()).
  return waitForIPv4Address();
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

// Setzt Fallback-DNS-Server, falls DHCP keinen brauchbaren DNS geliefert hat.
// Viele Android-Hotspots verschicken per DHCP keine DNS-Option; Linux-Clients
// fallen dann stillschweigend auf das Gateway zurück, lwIP tut das nicht.
// Deshalb hier: 1) Gateway des aktuellen Netzes (dynamisch ausgelesen, passt
// sich damit jedem Hotspot/Router an), 2)+3) öffentliche DNS-Server
// (netzunabhängig, funktionieren überall mit Internetzugang).
static void setFallbackDnsServers() {
  esp_netif_ip_info_t info;
  if (esp_netif_get_ip_info(sta_netif, &info) == ESP_OK && info.gw.addr != 0) {
    ip_addr_t gw;
    ip_addr_set_ip4_u32(&gw, info.gw.addr);   // Gateway = DNS-Forwarder des Hotspots
    dns_setserver(0, &gw);
  }

  ip_addr_t dns1;
  IP_ADDR4(&dns1, 8, 8, 8, 8);   // Google Public DNS
  dns_setserver(1, &dns1);

  ip_addr_t dns2;
  IP_ADDR4(&dns2, 1, 1, 1, 1);   // Cloudflare Public DNS
  dns_setserver(2, &dns2);
}

bool resolveDNS() {
  struct addrinfo hints = {};
  struct addrinfo* result = nullptr;

  // Hinweise für die DNS-Auflösung: wir wollen IPv4 (AF_INET) + TCP.
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_protocol = IPPROTO_TCP;

  // Portnummer in einen String umwandeln (getaddrinfo erwartet Text).
  char portString[8];
  snprintf(portString, sizeof(portString), "%d", port);

  // Hostname + Port in eine IP-Adresse auflösen (DNS-Anfrage).
  int err = getaddrinfo(server_address, portString, &hints, &result);
  if (err != 0 || result == nullptr) {
    // Manche Netze (v. a. Android-Hotspots) liefern per DHCP keinen DNS-Server.
    // Dann schlägt die Auflösung sofort fehl. Als Fallback setzen wir das
    // aktuelle Gateway sowie öffentliche DNS-Server und versuchen es erneut.
    ESP_LOGW(TAG, "DNS failed, error=%d, retrying with fallback DNS", err);
    setFallbackDnsServers();
    err = getaddrinfo(server_address, portString, &hints, &result);
  }
  if (err != 0 || result == nullptr) {
    ESP_LOGE(TAG, "DNS failed, error=%d", err);
    return false;
  }

  // Alle gefundenen Adressen ausgeben (Debug).
  int i = 0;
  for (struct addrinfo* p = result; p != nullptr; p = p->ai_next) {
    char addrStr[INET_ADDRSTRLEN] = {};
    struct sockaddr_in* addr4 = reinterpret_cast<struct sockaddr_in*>(p->ai_addr);
    inet_ntop(AF_INET, &(addr4->sin_addr), addrStr, sizeof(addrStr));  // Binär -> Text
    ESP_LOGI(TAG, "DNS result %d: %s:%d", i++, addrStr, ntohs(addr4->sin_port));  // ntohs: Port in lesbare Reihenfolge
  }

  // Erste Adresse übernehmen (für den späteren TCP-connect).
  memcpy(&server_addr, result->ai_addr, result->ai_addrlen);
  server_addr_len = result->ai_addrlen;

  ESP_LOGI(TAG, "DNS OK");
  freeaddrinfo(result);  // vom DNS belegten Speicher wieder freigeben
  return true;
}

bool connectTCP(int retry_counter) {
  ESP_LOGI(TAG, "Connecting TCP...");

  // TCP-Socket öffnen: IPv4 (AF_INET), verbindungsorientiert (SOCK_STREAM).
  sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (sock < 0) {
    ESP_LOGE(TAG, "socket failed: %s", strerror(errno));  // strerror: Fehlercode -> Text
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
    ESP_LOGE(TAG, "connect failed: %s (%d)", strerror(errno), errno);
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
    ESP_LOGE(TAG, "connect timeout");
    close(sock);
    sock = -1;
    return false;
  }

  // Prüfen, ob die Verbindung wirklich zustande kam (SO_ERROR muss 0 sein).
  int so_error = 0;
  socklen_t so_len = sizeof(so_error);
  getsockopt(sock, SOL_SOCKET, SO_ERROR, &so_error, &so_len);
  if (so_error != 0) {
    ESP_LOGE(TAG, "connect failed: %s (%d)", strerror(so_error), so_error);
    close(sock);
    sock = -1;
    return false;
  }

  // Socket wieder blockierend schalten (für die normale Nutzung).
  fcntl(sock, F_SETFL, flags);

  ESP_LOGI(TAG, "TCP connected");

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
    ESP_LOGE(TAG, "select failed: %s (%d)", socketErrText(errno), errno);
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
      ESP_LOGE(TAG, "send failed: %s (%d)", socketErrText(errno), errno);  // errno = letzter Fehlercode
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
      ESP_LOGE(TAG, "recv failed: %s (%d)", socketErrText(errno), errno);  // errno = letzter Fehlercode
      disconnectTCP();
      state = TCP_CONNECT;
      return false;
    }
  }

  return true;
}

// =============================================================================
// Paket zusammenbauen und senden (Header: status + msg_id + size + payload)
// =============================================================================
static bool sendPackageBytes(const Package& pkg) {
  std::vector<uint8_t> packet;
  packet.reserve(11 + pkg.payload.size());

  packet.push_back(pkg.status);                      // Byte 0: status
  for (int i = 7; i >= 0; i--)                       // Byte 1..8: msg_id (Big-Endian)
    packet.push_back((pkg.msg_id >> (i * 8)) & 0xFF);
  packet.push_back((pkg.size >> 8) & 0xFF);          // Byte 9: size (High)
  packet.push_back(pkg.size & 0xFF);                 // Byte 10: size (Low)
  packet.insert(packet.end(), pkg.payload.begin(), pkg.payload.end());

  return sendAll(packet.data(), packet.size());
}

// Bestätigung (Zustell-ACK) für eine empfangene server_msg_id an den Server.
static void sendAck(uint64_t server_msg_id) {
  Package ack;
  ack.status = 2;            // STATUS_ACK
  ack.msg_id = server_msg_id;
  ack.size = 0;
  ESP_LOGI(TAG, "SEND delivery ACK server_msg_id=%llu",
           (unsigned long long)server_msg_id);
  sendPackageBytes(ack);
}

// Registrierung: eigene MAC-Adresse an den Server schicken (nach jedem Verbinden).
void sendRegister() {
  Package reg;
  reg.status = 4;            // STATUS_REGISTER
  reg.msg_id = 0;
  reg.payload.assign(device_mac, device_mac + sizeof(device_mac));
  reg.size = reg.payload.size();
  ESP_LOGI(TAG, "SEND register MAC=%02x:%02x:%02x:%02x:%02x:%02x",
           device_mac[0], device_mac[1], device_mac[2],
           device_mac[3], device_mac[4], device_mac[5]);
  sendPackageBytes(reg);
}

// --- Dedupe: wurde diese server_msg_id schon verarbeitet? -------------------
static bool alreadyProcessed(uint64_t server_msg_id) {
  for (uint64_t id : processedIds)
    if (id == server_msg_id)
      return true;
  return false;
}

static void markProcessed(uint64_t server_msg_id) {
  if (processedIds.size() >= PROCESSED_MAX)
    processedIds.pop_front();
  processedIds.push_back(server_msg_id);
}

// --- Ausstehende Sendungen (Retry) ------------------------------------------
static void removePending(uint64_t client_msg_id) {
  for (auto it = pendingSends.begin(); it != pendingSends.end();) {
    if (it->pkg.msg_id == client_msg_id)
      it = pendingSends.erase(it);
    else
      ++it;
  }
}

// Sendet alle ausstehenden Nachrichten erneut (mit derselben client_msg_id!),
// damit der Server Duplikate korrekt dedupliziert. Wird NACH einem Reconnect
// aufgerufen: Innerhalb einer bestehenden TCP-Verbindung ist kein Retry nötig
// (TCP garantiert zuverlässige Zustellung) – erst die neue Verbindung muss die
// noch unbestätigten Nachrichten erneut zustellen.
void resendPendingSends() {
  uint32_t now = nowMs();
  for (auto& ps : pendingSends) {
    if (!sendPackageBytes(ps.pkg)) {
      // Socket wieder defekt -> abbrechen, sendAll schaltet auf TCP_CONNECT um.
      return;
    }
    ps.last_send_ms = now;
    ps.retries++;
    ESP_LOGW(TAG, "RESEND client_msg_id=%llu (Reconnect, Versuch %d)",
             (unsigned long long)ps.pkg.msg_id, ps.retries);
  }
}

// =============================================================================
// Empfangen
// =============================================================================
void receivePackage() {
  Package incoming;

  // 11-Byte-Header lesen: status (1) + msg_id (8) + size (2).
  uint8_t header[11];
  if (!recvAll(header, sizeof(header)))
    return;

  incoming.status = header[0];
  uint64_t msg_id = 0;
  for (int i = 0; i < 8; i++)
    msg_id = (msg_id << 8) | header[1 + i];
  incoming.msg_id = msg_id;
  incoming.size = (uint16_t(header[9]) << 8) | header[10];

  incoming.payload.resize(incoming.size);
  if (!recvAll(incoming.payload.data(), incoming.size))
    return;

  // Annahme-ACK vom Server (status 2): ausstehende Sendung entfernen.
  if (incoming.status == 2) {
    ESP_LOGI(TAG, "RECV acceptance ACK client_msg_id=%llu -> aus pending entfernt",
             (unsigned long long)incoming.msg_id);
    removePending(incoming.msg_id);
    return;
  }

  // Keepalive-Antwort (status 0): ignorieren.
  if (incoming.status == 0) {
    ESP_LOGI(TAG, "RECV keepalive");
    return;
  }

  // Nachricht (status 1) oder Check-Echo (status 3): abspielen + bestätigen.
  if (incoming.status == 1 || incoming.status == 3) {
    ESP_LOGI(TAG, "RECV %s server_msg_id=%llu size=%u payload=%s",
             incoming.status == 1 ? "msg" : "check",
             (unsigned long long)incoming.msg_id, incoming.size,
             packageToText(incoming).c_str());
    if (alreadyProcessed(incoming.msg_id)) {
      // Bereits verarbeitet (verlorenes ACK) -> nur erneut bestätigen.
      ESP_LOGI(TAG, "  duplicate -> SEND delivery ACK erneut");
      sendAck(incoming.msg_id);
      return;
    }
    markProcessed(incoming.msg_id);
    if (!putPackageIntoQueue(playbackQueue, incoming))
      ESP_LOGW(TAG, "playbackQueue overflow");
    sendAck(incoming.msg_id);
    return;
  }

  ESP_LOGW(TAG, "Unbekannter Status: %u", incoming.status);
}

// =============================================================================
// Senden
// =============================================================================
void sendPackage() {
  Package outgoing;
  // Gibt es überhaupt ein zu sendendes Paket?
  if (uxQueueMessagesWaiting(sendQueue) == 0)
    return;
  if (!getPackageFromQueue(sendQueue, outgoing))
    return;

  // Neue, eindeutige client_msg_id vergeben (für Retry-Erkennung am Server).
  outgoing.msg_id = newClientMsgId();

  if (!sendPackageBytes(outgoing)) {
    // Socket defekt: Nachricht zurücklegen, nach Reconnect erneut senden.
    if (!putPackageIntoQueue(sendQueue, outgoing))
      ESP_LOGW(TAG, "sendQueue overflow");
    return;
  }

  ESP_LOGI(TAG, "SEND msg client_msg_id=%llu size=%u (wartet auf ACK)",
           (unsigned long long)outgoing.msg_id, outgoing.size);

  PendingSend ps;
  ps.pkg = outgoing;
  ps.last_send_ms = nowMs();
  ps.retries = 0;
  pendingSends.push_back(ps);
}
