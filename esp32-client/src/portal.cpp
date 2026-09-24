// =============================================================================
// portal.cpp  –  Hotspot/Config-Portal (AP + Webserver + Captive Portal)
//
// Im Hotspot-Modus (Drehschalter) startet der ESP32 einen offenen Access Point
// "Morse" und einen Webserver. Über die Seite lassen sich die WLAN-Zugangsdaten
// eintragen, speichern und ein WLAN-Scan durchführen. Ein DNS-Responder
// (Captive Portal) lenkt das Handy automatisch auf http://192.168.4.1.
// =============================================================================

#include "portal.h"

#include <esp_http_server.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <esp_wifi.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <lwip/sockets.h>
#include <lwip/inet.h>

#include <cctype>
#include <cstdio>
#include <string>
#include <string.h>

#include "app_state.h"
#include "config.h"
#include "network.h"
#include "wifi_config.h"

static const char* TAG = "portal";

static esp_netif_t* ap_netif = nullptr;
static httpd_handle_t server = nullptr;
static TaskHandle_t dns_task = nullptr;
static volatile bool dns_running = false;
static bool active = false;

// ---------------------------------------------------------------------
// Kleine Helfer
// ---------------------------------------------------------------------
static void htmlEscape(const char* s, std::string& out) {
  for (const char* p = s; *p; ++p) {
    switch (*p) {
      case '&': out += "&amp;"; break;
      case '<': out += "&lt;"; break;
      case '>': out += "&gt;"; break;
      case '"': out += "&quot;"; break;
      default: out += *p; break;
    }
  }
}

static int hexVal(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return 0;
}

static void urlDecode(std::string& s) {
  std::string out;
  out.reserve(s.size());
  for (size_t i = 0; i < s.size(); ++i) {
    char c = s[i];
    if (c == '+') {
      out += ' ';
    } else if (c == '%' && i + 2 < s.size() &&
               std::isxdigit((unsigned char)s[i + 1]) &&
               std::isxdigit((unsigned char)s[i + 2])) {
      out += (char)((hexVal(s[i + 1]) << 4) | hexVal(s[i + 2]));
      i += 2;
    } else {
      out += c;
    }
  }
  s = out;
}

static const char* stateText(ConnectionState s) {
  switch (s) {
    case WIFI_CONNECT: return "Verbinde mit WLAN…";
    case DNS_RESOLVE:  return "Löse Server auf…";
    case TCP_CONNECT:  return "Verbinde mit Server…";
    case RUNNING:      return "Betriebsbereit";
  }
  return "Unbekannt";
}

static void applyApConfig() {
  wifi_config_t ap_cfg = {};
  strlcpy(reinterpret_cast<char*>(ap_cfg.ap.ssid), AP_SSID, sizeof(ap_cfg.ap.ssid));
  ap_cfg.ap.ssid_len = strlen(AP_SSID);
  ap_cfg.ap.channel = AP_CHANNEL;
  ap_cfg.ap.max_connection = AP_MAX_CONN;
  ap_cfg.ap.authmode = WIFI_AUTH_OPEN;
  esp_wifi_set_config(WIFI_IF_AP, &ap_cfg);
}

// ---------------------------------------------------------------------
// Captive-Portal-DNS: beantwortet jede Anfrage mit 192.168.4.1
// ---------------------------------------------------------------------
static bool buildDnsResponse(const uint8_t* query, size_t qlen,
                             uint8_t* resp, size_t* rlen) {
  if (qlen < 12)
    return false;

  // Frage-Abschnitt finden: QNAME überspringen.
  size_t q = 12;
  while (q < qlen) {
    uint8_t len = query[q];
    if (len == 0) { q++; break; }
    if ((len & 0xC0) == 0xC0) { q += 2; break; }  // Komprimierungs-Zeiger
    q += 1 + len;
    if (q + 4 > qlen) return false;
  }
  if (q + 4 > qlen)
    return false;

  uint16_t qtype = (query[q] << 8) | query[q + 1];
  size_t qend = q + 4;

  bool answerA = (qtype == 1);  // nur A-Records beantworten

  memcpy(resp, query, 12);
  resp[0] = query[0];
  resp[1] = query[1];
  resp[2] = 0x81;  // QR=1, RD=1
  resp[3] = 0x80;  // RA=1, RCODE=0
  resp[4] = query[4];
  resp[5] = query[5];
  resp[6] = 0x00;
  resp[7] = answerA ? 0x01 : 0x00;  // ANCOUNT
  resp[8] = 0x00; resp[9] = 0x00;   // NSCOUNT
  resp[10] = 0x00; resp[11] = 0x00; // ARCOUNT

  memcpy(resp + 12, query + 12, qend - 12);

  size_t off = qend;
  if (answerA) {
    resp[off++] = 0xC0; resp[off++] = 0x0C;  // Name -> Zeiger auf Offset 12
    resp[off++] = 0x00; resp[off++] = 0x01;  // TYPE A
    resp[off++] = 0x00; resp[off++] = 0x01;  // CLASS IN
    resp[off++] = 0x00; resp[off++] = 0x00;
    resp[off++] = 0x00; resp[off++] = 60;    // TTL = 60 s
    resp[off++] = 0x00; resp[off++] = 0x04;  // RDLENGTH = 4
    resp[off++] = 192; resp[off++] = 168;
    resp[off++] = 4;   resp[off++] = 1;      // 192.168.4.1
  }
  *rlen = off;
  return true;
}

static void dnsServerTask(void* arg) {
  int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (sock < 0) {
    ESP_LOGE(TAG, "DNS-Socket konnte nicht erstellt werden");
    vTaskDelete(nullptr);
    return;
  }

  struct sockaddr_in addr = {};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(53);
  if (bind(sock, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
    ESP_LOGE(TAG, "DNS-Bind auf Port 53 fehlgeschlagen");
    close(sock);
    vTaskDelete(nullptr);
    return;
  }

  timeval tv = { 1, 0 };  // 1 s Timeout -> Task endet beim Stop sauber
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  uint8_t query[512];
  uint8_t resp[512];
  while (dns_running) {
    struct sockaddr_in from = {};
    socklen_t fromlen = sizeof(from);
    int n = recvfrom(sock, query, sizeof(query), 0,
                     reinterpret_cast<struct sockaddr*>(&from), &fromlen);
    if (n < 12)
      continue;  // Timeout oder ungültige Anfrage

    size_t rlen = 0;
    if (buildDnsResponse(query, (size_t)n, resp, &rlen))
      sendto(sock, resp, rlen, 0,
             reinterpret_cast<struct sockaddr*>(&from), fromlen);
  }

  close(sock);
  vTaskDelete(nullptr);
}

static void startDnsServer() {
  if (dns_task != nullptr)
    return;
  dns_running = true;
  xTaskCreate(dnsServerTask, "dns_captive", 4096, nullptr, 3, &dns_task);
}

static void stopDnsServer() {
  if (dns_task == nullptr)
    return;
  dns_running = false;
  // Max. ~2 s warten, bis der Task aus recvfrom (1-s-Timeout) raus ist.
  for (int i = 0; i < 20 && eTaskGetState(dns_task) != eDeleted; i++)
    vTaskDelay(pdMS_TO_TICKS(100));
  if (eTaskGetState(dns_task) != eDeleted)
    vTaskDelete(dns_task);
  dns_task = nullptr;
}

// ---------------------------------------------------------------------
// Webserver: HTML-Seite
// ---------------------------------------------------------------------
static std::string buildHtmlPage() {
  std::string h;
  h.reserve(2048);

  h += "<!doctype html>\n<html lang=\"de\">\n<head>\n";
  h += "<meta charset=\"utf-8\">\n";
  h += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n";
  h += "<title>Morse Konfiguration</title>\n";
  h += "<style>body{font-family:system-ui,sans-serif;background:#111;color:#eee;margin:0;padding:16px}h1{font-size:20px}label{display:block;margin-top:12px;font-size:14px}input,button{font-size:16px;padding:8px;width:100%;box-sizing:border-box;margin-top:4px}button{background:#2a7;color:#fff;border:0;border-radius:6px;margin-top:16px}#status{margin-top:16px;padding:10px;background:#222;border-radius:6px;font-size:14px}</style>\n";
  h += "</head>\n<body>\n";
  h += "<h1>Morse – WLAN einrichten</h1>\n";
  h += "<form action=\"/save\" method=\"post\">\n";

  h += "<label>WLAN 1 (Name)</label>\n<input name=\"ssid1\" value=\"";
  htmlEscape(getSsid1(), h);
  h += "\" autocomplete=\"off\">\n";

  h += "<label>WLAN 1 (Passwort)</label>\n<input name=\"pw1\" type=\"text\" value=\"";
  htmlEscape(getPassword1(), h);
  h += "\" autocomplete=\"off\">\n";

  h += "<label>WLAN 2 (Name, optional)</label>\n<input name=\"ssid2\" value=\"";
  htmlEscape(getSsid2(), h);
  h += "\" autocomplete=\"off\">\n";

  h += "<label>WLAN 2 (Passwort, optional)</label>\n<input name=\"pw2\" type=\"text\" value=\"";
  htmlEscape(getPassword2(), h);
  h += "\" autocomplete=\"off\">\n";

  h += "<button type=\"submit\">Speichern &amp; verbinden</button>\n";
  h += "</form>\n";

  char ip[32] = { 0 };
  wifiStaIp(ip, sizeof(ip));
  h += "<div id=\"status\">Status: ";
  h += stateText(state);
  h += wifiIsConnected() ? " – verbunden" : " – nicht verbunden";
  if (ip[0] != '\0') {
    h += " (IP: ";
    h += ip;
    h += ")";
  }
  h += "</div>\n";

  h += "</body>\n</html>";
  return h;
}



// ---------------------------------------------------------------------
// Webserver: Handler
// ---------------------------------------------------------------------
static esp_err_t rootHandler(httpd_req_t* req) {
  std::string page = buildHtmlPage();
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");  // keine alte Seite cachen
  httpd_resp_sendstr(req, page.c_str());
  return ESP_OK;
}

// Captive-Portal: Alle unbekannten Pfade beantworten wir DIREKT mit der
// Konfigurationsseite (HTTP 200). So erkennen Android ("generate_204" -> kein
// 204), Apple ("hotspot-detect.html" -> kein "Success"), Windows
// ("connecttest.txt" -> nicht "Microsoft Connect Test") und Linux zuverlässig,
// dass es sich um ein Captive Portal handelt – unabhängig davon, ob der Client
// Redirects folgt oder nicht.

static void parseForm(const char* body, std::string& s1, std::string& p1,
                      std::string& s2, std::string& p2) {
  const char* p = body;
  while (p != nullptr && *p != '\0') {
    const char* amp = strchr(p, '&');
    std::string pair = (amp != nullptr) ? std::string(p, amp - p) : std::string(p);

    size_t eq = pair.find('=');
    std::string key = (eq == std::string::npos) ? pair : pair.substr(0, eq);
    std::string val = (eq == std::string::npos) ? "" : pair.substr(eq + 1);
    urlDecode(key);
    urlDecode(val);

    if (key == "ssid1") s1 = val;
    else if (key == "pw1") p1 = val;
    else if (key == "ssid2") s2 = val;
    else if (key == "pw2") p2 = val;

    if (amp == nullptr)
      break;
    p = amp + 1;
  }
}

static esp_err_t saveHandler(httpd_req_t* req) {
  char body[1024] = { 0 };
  int total = 0;
  while (total < (int)sizeof(body) - 1) {
    int n = httpd_req_recv(req, body + total, sizeof(body) - 1 - total);
    if (n <= 0)
      break;
    total += n;
  }
  body[total] = '\0';

  std::string s1, p1, s2, p2;
  parseForm(body, s1, p1, s2, p2);

  if (saveWifiConfig(s1.c_str(), p1.c_str(), s2.c_str(), p2.c_str())) {
    wifiReconnectRequested = true;  // ConnectionTask verbindet mit den neuen Daten
    // Redirect zur Startseite – funktioniert auch ohne JavaScript.
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "http://192.168.4.1/");
    httpd_resp_sendstr(req, "");
  } else {
    httpd_resp_set_status(req, "500 Internal Server Error");
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    httpd_resp_sendstr(req, "Speichern fehlgeschlagen");
  }
  return ESP_OK;
}

static void addHandler(const char* uri, httpd_method_t method,
                       esp_err_t (*fn)(httpd_req_t*)) {
  httpd_uri_t u = {};
  u.uri = uri;
  u.method = method;
  u.handler = fn;
  httpd_register_uri_handler(server, &u);
}

static void startHttpServer() {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.max_uri_handlers = 4;
  cfg.lru_purge_enable = true;
  cfg.uri_match_fn = httpd_uri_match_wildcard;
  if (httpd_start(&server, &cfg) != ESP_OK) {
    ESP_LOGE(TAG, "HTTP-Server konnte nicht gestartet werden");
    server = nullptr;
    return;
  }

  addHandler("/", HTTP_GET, rootHandler);
  addHandler("/save", HTTP_POST, saveHandler);
  addHandler("/*", static_cast<httpd_method_t>(HTTP_ANY), rootHandler);  // Captive Portal: alles -> Seite
}

static void stopHttpServer() {
  if (server != nullptr) {
    httpd_stop(server);
    server = nullptr;
  }
}

void portalStart() {
  if (active)
    return;

  if (ap_netif == nullptr)
    ap_netif = esp_netif_create_default_wifi_ap();  // inkl. DHCP + 192.168.4.1

  esp_wifi_stop();
  esp_wifi_set_mode(WIFI_MODE_APSTA);
  applyApConfig();
  esp_wifi_start();

  startHttpServer();
  startDnsServer();
  active = true;

  ESP_LOGI(TAG, "Portal aktiv: AP '%s' (offen) unter 192.168.4.1", AP_SSID);
}

void portalStop() {
  if (!active)
    return;

  stopHttpServer();
  stopDnsServer();

  esp_wifi_stop();
  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_start();

  active = false;
  ESP_LOGI(TAG, "Portal gestoppt");
}

bool portalIsActive() { return active; }

