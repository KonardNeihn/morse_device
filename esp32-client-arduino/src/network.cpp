// =============================================================================
// network.cpp  –  WLAN, DNS und TCP (niedrige Netzwerk-Ebene)
//
// Enthält die eigentlichen Arduino-WiFi-/WiFiClient-Aufrufe:
//   - WLAN initialisieren und verbinden (inkl. IPv4-Adresse (DHCP) abwarten)
//   - Hostname per DNS auflösen (übernimmt WiFiClient.connect() intern)
//   - TCP-Verbindung öffnen, verbinden, prüfen
//   - Pakete senden/empfangen (Header + Payload)
//
// Das Netzwerk-Protokoll eines Pakets (muss zum Server passen):
//   Byte 0     : status (0=keepalive, 1=Nachricht, 2=ACK, 3=check, 4=register)
//   Byte 1..8  : msg_id (64 Bit, Big-Endian; Bedeutung je Richtung)
//   Byte 9..10 : Größe des Payloads (16 Bit, Big-Endian)
//   Byte 11..  : Payload (Morse-Bytes bzw. beim Register die 6-Byte-MAC)
// =============================================================================

#include "network.h"
#include "app.h"
#include "config.h"
#include "esp_wifi_types_generic.h"
#include "freertos/projdefs.h"
#include "package.h"
#include "log.h"
#include "storage.h"

#include <WiFi.h>
#include <WiFiMulti.h>
#include <Arduino.h>

#include <cstdint>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <lwip/sockets.h>   // SO_KEEPALIVE, SOL_SOCKET, IPPROTO_TCP, TCP_KEEPIDLE, ...

#include <algorithm>
#include <vector>
#include <map>
#include <set>

static const char* TAG = "network";

// ------------------------- Statics ------------------------------------------
// Eine einzige, globale TCP-Verbindung. Ersetzt den rohen lwIP-Socket `sock`.
static WiFiMulti wifiMulti;
static WiFiClient client;

// Timeouts der Netzwerk-Schicht (in Millisekunden – Arduino-APIs rechnen in ms).
static constexpr int CONNECT_TIMEOUT_MS = 5000;    // TCP-connect() wartet max. so lange
static constexpr int SOCKET_IO_TIMEOUT_MS = 5000;  // recv()/send() warten max. so lange
static constexpr int RECV_POLL_DELAY_MS = 10;      // Pause zwischen recv()-Versuchen

// ------------------------- Zustand & Puffer ---------------------------------
// Eigene MAC-Adresse (Identität gegenüber dem Server).
static uint8_t device_mac[6] = { 0 };

// Liest die MAC-Adresse der STA-Schnittstelle aus und legt sie in device_mac ab.
static void readDeviceMac() {
  WiFi.macAddress(device_mac);   // füllt 6 Bytes mit der STA-MAC-Adresse
  LOGI(TAG, "MAC: %02x:%02x:%02x:%02x:%02x:%02x",
       device_mac[0], device_mac[1], device_mac[2],
       device_mac[3], device_mac[4], device_mac[5]);
}

// Ausstehende, noch nicht bestätigte Sendungen (für Retry bei fehlendem ACK).
struct PendingSend {
  Package pkg;         // enthält status, msg_id (client_msg_id) und payload
  uint32_t last_send_ms;
  int retries;
};

static std::map<uint64_t, PendingSend> pendingSends;

// Zuletzt verarbeitete server_msg_ids (Dedupe gegen doppeltes Zustellen).
static std::set<uint64_t> processedIds;
// server_msg_ids, für die das Zustell-ACK bereits gesendet wurde.
static std::set<uint64_t> ackedIds;

// ------------------------- Hilfsfunktionen ----------------------------------
static uint32_t nowMs() {
  return millis();
}

// client_msg_id: eindeutige ID für ausgehende Nachrichten. Zeitbasiert
// (Mikrosekunden seit Systemstart, monoton steigend) – dadurch ist kein
// Flash-Zugriff nötig und der NVS wird nicht pro Nachricht verschlissen.
static uint64_t newClientMsgId() {
  // micros() liefert nur 32 Bit und läuft nach ~71 min über. Damit die ID
  // trotzdem innerhalb einer Boot-Sitzung monoton bleibt, merken wir uns den
  // letzten Wert und zählen notfalls um 1 weiter.
  static uint32_t last = 0;
  uint32_t now = micros();
  if (now <= last)
    now = last + 1;
  last = now;
  return (uint64_t)now;
}

// Lesbaren Namen für ein WLAN-Event liefern (nur für Log-Ausgaben).
static const char* wifiEventName(WiFiEvent_t id) {
  switch (id) {
    case ARDUINO_EVENT_WIFI_READY:               return "WIFI_READY";
    case ARDUINO_EVENT_WIFI_SCAN_DONE:           return "SCAN_DONE";
    case ARDUINO_EVENT_WIFI_STA_START:           return "STA_START";
    case ARDUINO_EVENT_WIFI_STA_STOP:            return "STA_STOP";
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:       return "STA_CONNECTED";
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:    return "STA_DISCONNECTED";
    case ARDUINO_EVENT_WIFI_STA_AUTHMODE_CHANGE: return "STA_AUTHMODE_CHANGE";
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:          return "STA_GOT_IP";
    case ARDUINO_EVENT_WIFI_STA_GOT_IP6:         return "STA_GOT_IP6";
    case ARDUINO_EVENT_WIFI_STA_LOST_IP:         return "STA_LOST_IP";
    case ARDUINO_EVENT_WIFI_AP_START:            return "AP_START";
    case ARDUINO_EVENT_WIFI_AP_STOP:             return "AP_STOP";
    case ARDUINO_EVENT_WIFI_AP_STACONNECTED:     return "AP_STACONNECTED";
    case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:  return "AP_STADISCONNECTED";
    default: return "UNKNOWN";
  }
}

// Lesbaren Text für einen WLAN-Abbruchgrund liefern.
const char* disconnectReason(uint8_t reason) {
  switch (reason) {
    case WIFI_REASON_UNSPECIFIED:                    return "UNSPECIFIED";
    case WIFI_REASON_AUTH_EXPIRE:                    return "AUTH_EXPIRE";
    case WIFI_REASON_AUTH_LEAVE:                     return "AUTH_LEAVE";
    case WIFI_REASON_DISASSOC_DUE_TO_INACTIVITY:     return "DISASSOC_DUE_TO_INACTIVITY";
    case WIFI_REASON_ASSOC_TOOMANY:                  return "ASSOC_TOOMANY";
    case WIFI_REASON_CLASS2_FRAME_FROM_NONAUTH_STA:  return "CLASS2_FRAME_FROM_NONAUTH_STA";
    case WIFI_REASON_CLASS3_FRAME_FROM_NONASSOC_STA: return "CLASS3_FRAME_FROM_NONASSOC_STA";
    case WIFI_REASON_ASSOC_LEAVE:                    return "ASSOC_LEAVE";
    case WIFI_REASON_ASSOC_NOT_AUTHED:               return "ASSOC_NOT_AUTHED";
    case WIFI_REASON_DISASSOC_PWRCAP_BAD:            return "DISASSOC_PWRCAP_BAD";
    case WIFI_REASON_DISASSOC_SUPCHAN_BAD:           return "DISASSOC_SUPCHAN_BAD";
    case WIFI_REASON_IE_INVALID:                     return "IE_INVALID";
    case WIFI_REASON_MIC_FAILURE:                    return "MIC_FAILURE";
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:         return "4WAY_HANDSHAKE_TIMEOUT";
    case WIFI_REASON_GROUP_KEY_UPDATE_TIMEOUT:       return "GROUP_KEY_UPDATE_TIMEOUT";
    case WIFI_REASON_STA_LEAVING:                    return "STA_LEAVING";
    case WIFI_REASON_BEACON_TIMEOUT:                 return "BEACON_TIMEOUT";
    case WIFI_REASON_NO_AP_FOUND:                    return "NO_AP_FOUND";
    case WIFI_REASON_AUTH_FAIL:                      return "AUTH_FAIL";
    case WIFI_REASON_ASSOC_FAIL:                     return "ASSOC_FAIL";
    case WIFI_REASON_HANDSHAKE_TIMEOUT:              return "HANDSHAKE_TIMEOUT";
    case WIFI_REASON_CONNECTION_FAIL:                return "CONNECTION_FAIL";
    default: return "UNKNOWN";
  }
}

// WLAN-Event-Handler (läuft im Arduino-Network-Event-Task): protokolliert
// Ereignisse (inkl. Abbruchgrund beim Trennen).
static void onWifiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
    uint8_t reason = info.wifi_sta_disconnected.reason;
    LOGW(TAG, "Disconnected: %s (%u)", disconnectReason(reason), reason);

    // "Endgültige" Fehlergründe: Weiterwarten auf dieselbe SSID bringt nichts
    // (AP nicht da bzw. Passwort falsch). Das Flag lässt connectWifi() sofort
    // aufgeben, damit schnell das Fallback-Netz probiert wird.
    switch (reason) {
      case WIFI_REASON_NO_AP_FOUND:
      case WIFI_REASON_AUTH_FAIL:
      case WIFI_REASON_ASSOC_FAIL:
      case WIFI_REASON_HANDSHAKE_TIMEOUT:
      case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
        LOGV("wifi stack", "...");
        break;
      default:
        break;  // harmlos (z. B. eigener WiFi.disconnect()) -> ignorieren
    }
  } else {
    LOGV(TAG, "WiFi event: %s (%d)", wifiEventName(event), (int)event);
  }
}

// ------------------------- WLAN ---------------------------------------------
void configureWifiCredentials() {
  wifiMulti.APlistClean();
  // Ohne gespeicherte WLAN-Daten gibt es nichts zu verbinden
  if (!hasWifiConfig()) {
    LOGI(TAG, "No credentials WiFi set");
    return;
  }
  if (getSsid1()[0]) wifiMulti.addAP(getSsid1(), getPassword1());
  if (getSsid2()[0]) wifiMulti.addAP(getSsid2(), getPassword2());
  LOGI(TAG, "wifi credentials configured");
}

void wifiInit() {
  WiFi.mode(HOTSPOT_MODE ? WIFI_AP_STA : WIFI_STA);
  //WiFi.setScanMethod(WIFI_FAST_SCAN);
  //WiFi.setSleep(true);
  //WiFi.setAutoReconnect(true);
  configureWifiCredentials();

  // WLAN-Events für Log-Ausgaben + Abbruch-Merker registrieren.
  WiFi.onEvent(onWifiEvent);

  // Eigene MAC-Adresse als Geräte-Identität auslesen.
  readDeviceMac();

  LOGI(TAG, "WLAN initialisiert");
}

bool wifiIsConnected() { return WiFi.status()==WL_CONNECTED; }

int wifiRssi() { return WiFi.RSSI(); }

void connectWifi(){ wifiMulti.run(); }

// ------------------------- TCP ----------------------------------------------
static void enableTcpKeepalive() {
  int enable = 1;
  client.setSocketOption(SOL_SOCKET, SO_KEEPALIVE, &enable, sizeof(enable));

  int idle = TCP_KEEPALIVE_IDLE_S;
  client.setSocketOption(IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));

  int interval = TCP_KEEPALIVE_INTERVAL_S;
  client.setSocketOption(IPPROTO_TCP, TCP_KEEPINTVL, &interval, sizeof(interval));

  int count = TCP_KEEPALIVE_COUNT;
  client.setSocketOption(IPPROTO_TCP, TCP_KEEPCNT, &count, sizeof(count));
}

void disconnectTCP() { client.stop(); }

bool connectTCP() {
  LOGI(TAG, "Connecting TCP to %s:%d ...", server_address, port);

  // WiFiClient.connect(host, port, timeout_ms) löst den Hostnamen intern per
  // DNS auf und stellt die TCP-Verbindung her.
  if (!client.connect(server_address, port, CONNECT_TIMEOUT_MS)) {
    LOGW(TAG, "TCP connection failed");
    return false;
  }

  // Timeout für die Stream-Lesehelfer (readBytes etc.) – Arduino rechnet in ms.
  client.setTimeout(SOCKET_IO_TIMEOUT_MS);
  enableTcpKeepalive();
  LOGI(TAG, "TCP connected");
  return true;
}

bool tcpIsConnected() { return client.connected(); }

bool tcpAvailableMessage() { return client.available() > 0; }

// ------------------------- Byte-I/O -----------------------------------------
// Komplette Daten senden (bis wirklich alle Bytes geschickt sind).
static bool sendAll(const uint8_t* data, size_t len) {
  size_t sent = 0;

  // WiFiClient.write() kann (theoretisch) weniger schicken als angefordert.
  while (sent < len) {
    size_t n = client.write(data + sent, len - sent);
    if (n > 0) {
      sent += n;
    } else {
      // write() liefert 0 bei Fehler oder geschlossener Verbindung.
      LOGW(TAG, "send failed: peer disconnected");
      disconnectTCP();
      state = TCP_CONNECT;
      return false;
    }
  }
  return true;
}

// Exakt `bytesToRead` Bytes empfangen.
static bool recvAll(uint8_t* bytes, size_t bytesToRead) {
  size_t got = 0;
  uint32_t start = millis();

  // WiFiClient.read() ist nicht blockierend; wir pollen mit kleiner Pause und
  // brechen nach SOCKET_IO_TIMEOUT_MS ab (entspricht dem alten SO_RCVTIMEO).
  while (got < bytesToRead) {
    int n = client.read(bytes + got, bytesToRead - got);
    if (n > 0) {
      got += n;
      start = millis();  // Daten kamen -> Timeout zurücksetzen
      continue;
    }

    // n == 0: (noch) keine Daten; n < 0: Fehler im Empfangspuffer.
    if (!client.connected()) {
      LOGW(TAG, "recv failed: peer disconnected");
      disconnectTCP();
      state = TCP_CONNECT;
      return false;
    }

    if (millis() - start >= (uint32_t)SOCKET_IO_TIMEOUT_MS) {
      LOGW(TAG, "recv failed: timeout");
      disconnectTCP();
      state = TCP_CONNECT;
      return false;
    }

    // Noch verbunden, aber (noch) keine Daten -> anderen Tasks Zeit geben.
    vTaskDelay(pdMS_TO_TICKS(RECV_POLL_DELAY_MS));
  }

  return true;
}

// Komplettes Paket (11-Byte-Header + Payload) senden.
static bool sendPackageBytes(const Package& pkg) {
  std::vector<uint8_t> packet;
  packet.reserve(11 + pkg.payload.size());

  packet.push_back(pkg.status);                        // Byte 0: status
  for (int i = 7; i >= 0; i--)                         // Byte 1..8: msg_id (Big-Endian)
    packet.push_back((pkg.msg_id >> (i * 8)) & 0xFF);
  packet.push_back((pkg.size >> 8) & 0xFF);            // Byte 9: size (High)
  packet.push_back(pkg.size & 0xFF);                   // Byte 10: size (Low)
  packet.insert(packet.end(), pkg.payload.begin(), pkg.payload.end());

  return sendAll(packet.data(), packet.size());
}

// ------------------------- ACK & Registrierung ------------------------------
// Zustell-ACK: bestätigt den Empfang einer server_msg_id. Wird über die
// sendQueue geschickt, damit nur der ConnectionTask in den Socket schreibt.
// Aufgerufen vom PrintTask nach dem Druck.
void queueDeliveryAck(uint64_t server_msg_id) {
  Package ack;
  ack.status = 2;   // ACK
  ack.msg_id = server_msg_id;   // sendPackage() behält die msg_id bei status==2
  ack.size = 0;

  if (!putPackageIntoQueue(sendQueue, ack))
    LOGW(TAG, "sendQueue overflow (ACK server_msg_id=%llu)",
         (unsigned long long)server_msg_id);
}

// Registrierung: eigene MAC-Adresse an den Server schicken (nach jedem Verbinden).
void sendRegister() {
  Package reg;
  reg.status = 4;            // STATUS_REGISTER
  reg.msg_id = 0;
  reg.payload.assign(device_mac, device_mac + sizeof(device_mac));
  reg.size = reg.payload.size();
  LOGI(TAG, "SEND register MAC=%02x:%02x:%02x:%02x:%02x:%02x",
       device_mac[0], device_mac[1], device_mac[2],
       device_mac[3], device_mac[4], device_mac[5]);
  sendPackageBytes(reg);
}

// ------------------------- Dedupe & Pending ---------------------------------
static bool alreadyProcessed(uint64_t server_msg_id) { return processedIds.count(server_msg_id); }

static void markProcessed(uint64_t server_msg_id) { processedIds.insert(server_msg_id); }

static bool alreadyAcked(uint64_t server_msg_id) { return ackedIds.count(server_msg_id); }

static void markAcked(uint64_t server_msg_id) { ackedIds.insert(server_msg_id); }

static void insertPending(PendingSend ps) { pendingSends[ps.pkg.msg_id] = ps; }

static void removePending(uint64_t client_msg_id) { pendingSends.erase(client_msg_id); }

void resendPendingSends() {
  uint32_t now = nowMs();
  for (auto& [msg_id, ps] : pendingSends) {
    if (!sendPackageBytes(ps.pkg)) {
      // Socket wieder defekt -> abbrechen; sendAll schaltet auf TCP_CONNECT um.
      return;
    }
    ps.last_send_ms = now;
    ps.retries++;
    LOGW(TAG, "RESEND client_msg_id=%llu (Reconnect, Versuch %d)",
         (unsigned long long)ps.pkg.msg_id, ps.retries);
  }
}

// ------------------------- Protokoll ----------------------------------------
// Ein Paket vom Server empfangen und weiterverarbeiten.
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
    LOGI(TAG, "RECV acceptance ACK client_msg_id=%llu -> aus pending entfernt",
         (unsigned long long)incoming.msg_id);
    removePending(incoming.msg_id);
    return;
  }

  // Keepalive-Antwort (status 0): ignorieren.
  if (incoming.status == 0) {
    LOGI(TAG, "RECV keepalive");
    return;
  }

  // Nachricht (status 1) oder Check-Echo (status 3): abspielen + bestätigen.
  if (incoming.status == 1 || incoming.status == 3) {
    LOGI(TAG, "RECV %s server_msg_id=%llu size=%u payload=%s",
         incoming.status == 1 ? "msg" : "check",
         (unsigned long long)incoming.msg_id, (unsigned)incoming.size,
         packageToText(incoming).c_str());
    if (alreadyProcessed(incoming.msg_id)) {
      if (alreadyAcked(incoming.msg_id)) {
        // ACK wurde schon gesendet, aber offenbar verloren -> erneut bestätigen.
        LOGI(TAG, "  duplicate -> SEND delivery ACK erneut");
        queueDeliveryAck(incoming.msg_id);
      } else {
        // Druck läuft noch -> ACK folgt nach dem Druck, hier nichts tun.
        LOGI(TAG, "  duplicate -> Druck läuft noch, ACK folgt später");
      }
      return;
    }
    markProcessed(incoming.msg_id);
    if (!putPackageIntoQueue(playbackQueue, incoming)) {
      // Queue voll (sollte durch Backpressure nicht passieren): dann sofort
      // bestätigen, damit die Nachricht nicht unbestätigt hängen bleibt.
      LOGW(TAG, "playbackQueue overflow -> ACK sofort senden");
      queueDeliveryAck(incoming.msg_id);
      markAcked(incoming.msg_id);
    }
    // KEIN ACK hier: das Zustell-ACK wird erst nach dem Druck gesendet
    // (PrintTask -> queueDeliveryAck -> sendQueue -> sendPackage).
    return;
  }

  LOGW(TAG, "Unbekannter Status: %u", (unsigned)incoming.status);
}

// Ein Paket aus der sendQueue an den Server senden.
void sendPackage() {
  Package outgoing;
  // Gibt es überhaupt ein zu sendendes Paket?
  if (uxQueueMessagesWaiting(sendQueue) == 0)
    return;
  if (!getPackageFromQueue(sendQueue, outgoing))
    return;

  // Neue, eindeutige client_msg_id vergeben (für Retry-Erkennung am Server).
  // Bei einem ACK die alte msg_id behalten (server_msg_id).
  if (outgoing.status != 2)
    outgoing.msg_id = newClientMsgId();

  if (!sendPackageBytes(outgoing)) {
    // Socket defekt: Nachricht zurücklegen, nach Reconnect erneut senden.
    LOGI(TAG, "couldnt send message %llu, returning message to send queue", (unsigned long long)outgoing.msg_id);
    if (!putPackageIntoQueue(sendQueue, outgoing))
      LOGW(TAG, "sendQueue overflow");
    return;
  }

  LOGI(TAG, "SEND msg client_msg_id=%llu size=%u (wartet auf ACK)",
       (unsigned long long)outgoing.msg_id, (unsigned)outgoing.size);

  PendingSend ps;
  ps.pkg = outgoing;
  ps.last_send_ms = nowMs();
  ps.retries = 0;
  insertPending(ps);
}
