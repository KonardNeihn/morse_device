#include "network.h"

#include <Arduino.h>
#include <esp_wifi.h>
#include <esp_netif.h>
#include <esp_netif_ip_addr.h>

#include <cstdio>
#include <cstring>

#include <errno.h>
#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include <lwip/inet.h>
#include "lwip/tcp.h"
#include "lwip/ip_addr.h"

#include "app_state.h"
#include "config.h"
#include "package.h"

// Interne Helper-Funktion der Arduino-WiFi-Bibliothek (definiert in
// WiFiGeneric.cpp); liefert das esp_netif_t* des jeweiligen Interfaces.
esp_netif_t* get_esp_interface_netif(esp_interface_t interface);

// ---------------------------------------------------------------------
// WLAN
// ---------------------------------------------------------------------

bool connectWifi(const char ssid[], const char password[]) {
  disconnectTCP();
  state = WIFI_CONNECT;
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  WiFi.mode(WIFI_STA);
  vTaskDelay(100 / portTICK_PERIOD_MS);

  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);

  int n = WiFi.scanNetworks();

  for (int i = 0; i < n; i++) {
    Serial.printf("%2d: %s RSSI=%d Ch=%d\n",
                  i,
                  WiFi.SSID(i).c_str(),
                  WiFi.RSSI(i),
                  WiFi.channel(i));
  }

  Serial.printf("Connecting to %s\n", ssid);

  WiFi.begin(ssid, password);

  n = WiFi.scanNetworks();

  for (int i = 0; i < n; i++) {
    Serial.printf("%2d: %s RSSI=%d Ch=%d\n",
                  i,
                  WiFi.SSID(i).c_str(),
                  WiFi.RSSI(i),
                  WiFi.channel(i));
  }

  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.printf("Couldn't connect to %s\n", ssid);
    return false;
  }

  Serial.println("WiFi OK");
  Serial.printf("IPv4: %s\n", WiFi.localIP().toString().c_str());

  state = WAIT_FOR_IP6;

  esp_netif_t* sta = get_esp_interface_netif(ESP_IF_WIFI_STA);

  // IPv6 erst NACH erfolgreicher Verbindung aktivieren: Das STA-Interface ist
  // erst dann "up". Solange es down ist, liefert
  // esp_netif_create_ip6_linklocal() ESP_FAIL und enableIpV6() meldet FAILED.
  bool ipv6Enabled = WiFi.enableIpV6();
  Serial.printf("IPv6 enable: %s\n", ipv6Enabled ? "OK" : "FAILED");

  // Auf die globale IPv6-Adresse warten (SLAAC via Router Advertisement).
  // Die link-lokale Adresse wird von enableIpV6() angelegt; daraufhin sendet
  // lwIP einen Router Solicitation und der Router antwortet mit einem RA.
  for (int i = 0; i < 20; i++) {
    Serial.printf("Waiting for IPv6 Address\n");
    if (sta != nullptr) {
      esp_ip6_addr_t addr;
      if (esp_netif_get_ip6_global(sta, &addr) == ESP_OK) {
        char addrStr[INET6_ADDRSTRLEN];
        inet_ntop(AF_INET6, addr.addr, addrStr, sizeof(addrStr));
        Serial.printf("IPv6 global: %s\n", addrStr);
        return true;
      }
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }

  Serial.printf("Didn't get IPv6 Address\n");
  return false;
}

const char* disconnectReason(uint8_t reason) {
  switch (reason) {
    case WIFI_REASON_UNSPECIFIED: return "UNSPECIFIED";
    case WIFI_REASON_AUTH_EXPIRE: return "AUTH_EXPIRE";
    case WIFI_REASON_AUTH_LEAVE: return "AUTH_LEAVE";
    case WIFI_REASON_ASSOC_EXPIRE: return "ASSOC_EXPIRE";
    case WIFI_REASON_ASSOC_TOOMANY: return "ASSOC_TOOMANY";
    case WIFI_REASON_NOT_AUTHED: return "NOT_AUTHED";
    case WIFI_REASON_NOT_ASSOCED: return "NOT_ASSOCED";
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

const char* wifiEventName(arduino_event_id_t event) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_READY: return "WIFI_READY";
    case ARDUINO_EVENT_WIFI_SCAN_DONE: return "SCAN_DONE";
    case ARDUINO_EVENT_WIFI_STA_START: return "STA_START";
    case ARDUINO_EVENT_WIFI_STA_STOP: return "STA_STOP";
    case ARDUINO_EVENT_WIFI_STA_CONNECTED: return "STA_CONNECTED";
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED: return "STA_DISCONNECTED";
    case ARDUINO_EVENT_WIFI_STA_GOT_IP: return "STA_GOT_IP";
    case ARDUINO_EVENT_WIFI_STA_LOST_IP: return "STA_LOST_IP";
    case ARDUINO_EVENT_WIFI_STA_GOT_IP6: return "STA_GOT_IP6";

    default:
      return "UNKNOWN";
  }
}

// ---------------------------------------------------------------------
// DNS + TCP
// ---------------------------------------------------------------------

void disconnectTCP() {
  if (sock >= 0)
    close(sock);
  sock = -1;
}

bool resolveDNS() {
  struct addrinfo hints = {};
  struct addrinfo* result = nullptr;

  hints.ai_family = AF_INET6;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_protocol = IPPROTO_TCP;

  char portString[8];
  snprintf(portString, sizeof(portString), "%d", port);

  int err = getaddrinfo(
    server_address,
    portString,
    &hints,
    &result);

  if (err != 0 || result == nullptr) {
    Serial.printf("IPv6 DNS failed, error=%d\n", err);
    return false;
  }

  // ALLE gefundenen Adressen ausgeben
  int i = 0;
  for (struct addrinfo* p = result; p != nullptr; p = p->ai_next) {
    char addrStr[INET6_ADDRSTRLEN] = {};

    struct sockaddr_in6* addr6 =
      reinterpret_cast<struct sockaddr_in6*>(p->ai_addr);

    inet_ntop(
      AF_INET6,
      &(addr6->sin6_addr),
      addrStr,
      sizeof(addrStr));

    Serial.printf(
      "DNS result %d: [%s]:%d\n",
      i++,
      addrStr,
      ntohs(addr6->sin6_port));
  }

  // erste Adresse übernehmen
  memcpy(
    &server_addr,
    result->ai_addr,
    result->ai_addrlen);

  server_addr_len = result->ai_addrlen;

  Serial.println("IPv6 DNS OK");

  freeaddrinfo(result);
  return true;
}

bool connectTCP(int retry_counter) {
  Serial.println("Connecting TCP...");

  // Socket erstellen: AF_INET6 → IPv6, SOCK_STREAM → TCP, IPPROTO_TCP → TCP-Protokoll
  sock = socket(AF_INET6, SOCK_STREAM, IPPROTO_TCP);

  if (sock < 0) {
    Serial.printf("IPv6 socket failed: %s\n", strerror(errno));
    return false;
  }

  if (connect(sock, (struct sockaddr*)&server_addr, server_addr_len) != 0) {
    Serial.printf("IPv6 connect failed: %s (%d)\n", strerror(errno), errno);

    close(sock);
    sock = -1;
    return false;
  }

  Serial.println("IPv6 TCP connected");

  // keep alive aktivieren
  int yes = 1;
  setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &yes, sizeof(yes));
  // wie viele Sekunden kein Verkehr, bis erstes keep alive
  int idle = 10;
  setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
  // falls erstes keep alive keine Antwort - wann das nächste in Sekunden
  int interval = 5;
  setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &interval, sizeof(interval));
  // wie oft keep alive wiederholen
  int count = 3;
  setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &count, sizeof(count));

  timeval timeout;

  timeout.tv_sec = 5;   // in Sekunden
  timeout.tv_usec = 0;  // in Mikrosekunden

  // receive timeout
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  // send timeout
  setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

  state = RUNNING;

  return true;
}

bool checkSocket(bool& readable, bool& writable) {
  // check if something's wrong / client connected?
  int err;
  int len = sizeof(err);
  getsockopt(sock, SOL_SOCKET, SO_ERROR, &err, (socklen_t*)&len);

  if (err) {
    Serial.printf("Socket error: %s (%d)\n", strerror(err), err);
    return false;
  }

  if (sock < 0) {
    Serial.printf("socket disconnected returning to DNS_RESOLVE\n");
    return false;
  }

  readable = false;
  writable = false;

  fd_set readfds;
  fd_set writefds;

  FD_ZERO(&readfds);
  FD_ZERO(&writefds);

  FD_SET(sock, &readfds);
  FD_SET(sock, &writefds);

  timeval tv{ 0, 0 };

  int ret = select(sock + 1,
                   &readfds,
                   &writefds,
                   nullptr,
                   &tv);

  if (ret < 0) {
    Serial.printf("select failed: %d\n", errno);
    return false;
  }

  readable = FD_ISSET(sock, &readfds);
  writable = FD_ISSET(sock, &writefds);

  return true;
}

// ---------------------------------------------------------------------
// Senden / Empfangen
// ---------------------------------------------------------------------

bool sendAll(const uint8_t* data, size_t len) {
  size_t sent = 0;

  while (sent < len) {
    int n = send(sock, data + sent, len - sent, 0);
    if (n > 0) {
      sent += n;
    } else if (n == 0) {
      Serial.println("peer disconnected");
      disconnectTCP();
      state = TCP_CONNECT;
      return false;
    } else {
      Serial.printf("send failed: %d\n", errno);
      disconnectTCP();
      state = TCP_CONNECT;
      return false;
    }
  }
  return true;
}

bool recvAll(uint8_t* bytes, size_t bytesToRead) {
  size_t got = 0;

  while (got < bytesToRead) {
    int n = recv(sock, bytes + got, bytesToRead - got, MSG_WAITALL);

    if (n > 0) {
      got += n;
    } else if (n == 0) {
      Serial.println("peer disconnected");
      disconnectTCP();
      state = TCP_CONNECT;
      return false;
    } else {
      Serial.printf("recv failed: %d\n", errno);
      disconnectTCP();
      state = TCP_CONNECT;
      return false;
    }
  }

  return true;
}

void receivePackage() {
  Package incoming;

  // recv header
  uint8_t header[3];
  if (!recvAll(header, 3))
    return;

  incoming.status = header[0];
  incoming.size = (uint16_t(header[1]) << 8) | header[2];

  // recv payload
  incoming.payload.resize(incoming.size);
  if (!recvAll(incoming.payload.data(), incoming.size))
    return;

  Serial.printf("%s\n", packageToText(incoming).c_str());

  if (!putPackageIntoQueue(playbackQueue, incoming))
    Serial.println("playbackQueue overflow");

  // confirmation package
  if (incoming.status == 2)
    return;

  // got my own server check message
  if (incoming.status == 3)
    return;

  // send receiving confirmation
  Package confirmation;
  confirmation.status = 2;
  confirmation.payload.push_back(0b11111111);
  confirmation.payload.push_back(0);
  confirmation.payload.push_back(0b11111111);
  confirmation.payload.push_back(0b11111111);
  confirmation.payload.push_back(0b11111111);
  confirmation.payload.push_back(0);
  confirmation.payload.push_back(0b11111111);
  confirmation.size = confirmation.payload.size();

  if (!putPackageIntoQueue(sendQueue, confirmation))
    Serial.println("sendQueue overflow");
}

void sendPackage() {
  Package outgoing;
  // package to send?
  if (uxQueueMessagesWaiting(sendQueue) == 0)
    return;
  getPackageFromQueue(sendQueue, outgoing);

  // gesamtes Paket erzeugen
  std::vector<uint8_t> packet;
  // header anhängen
  packet.push_back(outgoing.status);
  packet.push_back((outgoing.size >> 8) & 0xFF);  // Hochwertiges Byte von size
  packet.push_back(outgoing.size & 0xFF);         // Niedrigwertiges Byte von size
  // payload anhängen
  packet.insert(packet.end(), outgoing.payload.begin(), outgoing.payload.end());

  // Send package
  sendAll(packet.data(), packet.size());
}
