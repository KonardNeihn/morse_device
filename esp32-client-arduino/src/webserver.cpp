// =============================================================================
// webserver.cpp  –  Hotspot/Config-Portal (AP + Webserver + Captive Portal)
//
// Im Hotspot-Modus (Kippschalter) startet der ESP32 einen offenen Access Point
// "Morse" und einen Webserver. Über die Seite lassen sich die WLAN-Zugangsdaten
// eintragen, speichern und anzeigen. Ein DNS-Responder (Captive Portal) lenkt
// das Handy automatisch auf http://192.168.4.1.
//
// Arduino-Port der ESP-IDF-Version (portal.cpp):
//   - esp_http_server            -> WebServer.h
//   - handgerollter UDP-DNS-Task -> DNSServer.h
//   - esp_wifi/esp_netif         -> WiFi.softAP / WiFi.mode
// =============================================================================

#include "webserver.h"
#include "app.h"        // state, newWifiCredentials
#include "config.h"     // AP_SSID, AP_CHANNEL
#include "storage.h"    // saveWifiConfig, getSsid1/2, getPassword1/2
#include "network.h"    // wifiIsConnected, wifiRssi
#include "log.h"

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cstdio>   // snprintf
#include <string>

static const char* TAG = "webserver";

static WebServer server(80);
static DNSServer dnsServer;

static TaskHandle_t webserverTask = nullptr;
static volatile bool running = false;

// -------------------------------------------------------------------------
// Kleine Helfer
// -------------------------------------------------------------------------
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

static const char* stateText(ConnectionState s) {
  switch (s) {
    case WIFI_CONNECT: return "Verbinde mit WLAN…";
    case TCP_CONNECT:  return "Verbinde mit Server…";
    case RUNNING:      return "Betriebsbereit";
  }
  return "Unbekannt";
}

static int stateIndex(ConnectionState s) {
  switch (s) {
    case WIFI_CONNECT: return 0;
    case TCP_CONNECT:  return 1;
    case RUNNING:      return 2;
  }
  return -1;
}

// STA-IP als Text (leer, wenn nicht verbunden)
static std::string staIp() {
  if (wifiIsConnected()) {
    String ip = WiFi.localIP().toString();
    return std::string(ip.c_str());
  }
  return std::string();
}

// -------------------------------------------------------------------------
// HTML-Seite
// -------------------------------------------------------------------------
static std::string buildHtmlPage() {
  std::string h;
  h.reserve(2048);

  h += "<!doctype html>\n<html lang=\"de\">\n<head>\n";
  h += "<meta charset=\"utf-8\">\n";
  h += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n";
  h += "<title>Morse Konfiguration</title>\n";
  h += "<style>body{font-family:system-ui,sans-serif;background:#111;color:#eee;margin:0;padding:16px}h1{font-size:20px}label{display:block;margin-top:12px;font-size:14px}input,button{font-size:16px;padding:8px;width:100%;box-sizing:border-box;margin-top:4px}button{background:#2a7;color:#fff;border:0;border-radius:6px;margin-top:16px}#status{margin-top:16px;padding:10px;background:#222;border-radius:6px;font-size:14px}#fsm{display:flex;flex-wrap:wrap;align-items:center;gap:8px;margin-top:16px}.step{padding:8px 12px;background:#222;border:2px solid #444;border-radius:6px;font-size:14px}.step.active{background:#2a7;border-color:#6f6;color:#fff;font-weight:bold}.arrow{color:#888}</style>\n";
  h += "</head>\n<body>\n";
  h += "<h1>Morse – WLAN einrichten</h1>\n";
  h += "<form action=\"/save\" method=\"post\">\n";

  h += "<label>WLAN 1 (Name)</label>\n<input name=\"ssid1\" value=\"";
  htmlEscape(getSsid1(), h);
  h += "\" autocomplete=\"off\">\n";

  h += "<label>WLAN 1 (Passwort)</label>\n<input name=\"pw1\" type=\"password\" placeholder=\"leer lassen = behalten\" autocomplete=\"new-password\">\n";
  h += "<label><input type=\"checkbox\" name=\"pw1_clear\" style=\"width:auto\"> Offenes Netzwerk (kein Passwort)</label>\n";

  h += "<label>WLAN 2 (Name, optional)</label>\n<input name=\"ssid2\" value=\"";
  htmlEscape(getSsid2(), h);
  h += "\" autocomplete=\"off\">\n";

  h += "<label>WLAN 2 (Passwort, optional)</label>\n<input name=\"pw2\" type=\"password\" placeholder=\"leer lassen = behalten\" autocomplete=\"new-password\">\n";
  h += "<label><input type=\"checkbox\" name=\"pw2_clear\" style=\"width:auto\"> Offenes Netzwerk (kein Passwort)</label>\n";

  h += "<button type=\"submit\">Speichern &amp; verbinden</button>\n";
  h += "</form>\n";

  // Zustandsmaschine als Diagramm: 3 Zustände, aktiver hervorgehoben.
  h += "<div id=\"fsm\">\n";
  for (int i = 0; i < 3; i++) {
    h += "<div class=\"step";
    if (i == stateIndex(state)) h += " active";
    h += "\" data-state=\"";
    h += std::to_string(i);
    h += "\">";
    switch (i) {
      case 0: h += "WLAN"; break;
      case 1: h += "TCP"; break;
      case 2: h += "Betrieb"; break;
    }
    h += "</div>\n";
    if (i < 2) h += "<span class=\"arrow\">→</span>\n";
  }
  h += "</div>\n";

  std::string ip = staIp();
  h += "<div id=\"status\">Status: ";
  h += stateText(state);
  h += wifiIsConnected() ? " – verbunden" : " – nicht verbunden";
  if (!ip.empty()) {
    h += " (IP: ";
    h += ip;
    h += ")";
  }
  if (wifiIsConnected()) {
    h += " · RSSI: ";
    h += std::to_string(wifiRssi());
    h += " dBm";
  }
  h += "</div>\n";

  // Live-Aktualisierung der Zustandsmaschine und der Werte (ohne Reload).
  h += "<script>async function refresh(){try{const r=await fetch('/status');const s=await r.json();document.querySelectorAll('#fsm .step').forEach(function(el){el.classList.toggle('active',parseInt(el.dataset.state)===s.state_index);});var t='Status: '+s.state+(s.wifi?' – verbunden':' – nicht verbunden');if(s.ip)t+=' (IP: '+s.ip+')';if(s.rssi!==-127)t+=' · RSSI: '+s.rssi+' dBm';document.getElementById('status').textContent=t;}catch(e){}}setInterval(refresh,1000);refresh();</script>\n";

  h += "</body>\n</html>";
  return h;
}

// -------------------------------------------------------------------------
// Handler
// -------------------------------------------------------------------------
static void handleRoot() {
  std::string page = buildHtmlPage();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/html", page.c_str());
}

static void handleStatus() {
  std::string ip = staIp();
  char json[192];
  snprintf(json, sizeof(json),
           "{\"state_index\":%d,\"state\":\"%s\",\"wifi\":%s,\"ip\":\"%s\",\"rssi\":%d}",
           stateIndex(state), stateText(state),
           wifiIsConnected() ? "true" : "false",
           ip.c_str(), wifiRssi());
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

static void handleSave() {
  // WebServer hat die form-urlencoded-Daten bereits geparst + URL-dekodiert.
  String s1 = server.arg("ssid1");
  String p1 = server.arg("pw1");
  String s2 = server.arg("ssid2");
  String p2 = server.arg("pw2");
  bool clearPw1 = server.hasArg("pw1_clear");
  bool clearPw2 = server.hasArg("pw2_clear");

  // Leeres Passwort + kein "offenes Netz"-Häkchen -> bestehendes behalten.
  if (!clearPw1 && p1.length() == 0) p1 = getPassword1();
  if (!clearPw2 && p2.length() == 0) p2 = getPassword2();

  if (saveWifiConfig(s1.c_str(), p1.c_str(), s2.c_str(), p2.c_str())) {
    newWifiCredentials = true;   // ConnectionTask lädt neu + verbindet
    server.sendHeader("Location", "/", true);   // 303 See Other
    server.send(303, "text/plain", "");
  } else {
    server.send(500, "text/plain", "Speichern fehlgeschlagen");
  }
}

// -------------------------------------------------------------------------
// Service-Task: bedient HTTP + DNS (weil FreeRTOS statt loop()).
// -------------------------------------------------------------------------
static void webserverTaskFunc(void*) {
  while (running) {
    dnsServer.processNextRequest();
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  vTaskDelete(nullptr);
}

// Routen nur einmal registrieren (bei mehrfachem Start/Stop nicht anhäufen).
static void registerRoutes() {
  static bool done = false;
  if (done)
    return;
  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/save", HTTP_POST, handleSave);
  server.onNotFound(handleRoot);   // Captive Portal: alles -> Seite
  done = true;
}

// -------------------------------------------------------------------------
// Start / Stop
// -------------------------------------------------------------------------
void webserverStart() {
  if (running)
    return;

  WiFi.mode(WIFI_AP_STA);                    // STA bleibt aktiv
  WiFi.softAP(AP_SSID, nullptr, AP_CHANNEL); // offener AP

  dnsServer.start(53, "*", WiFi.softAPIP()); // Captive Portal DNS

  registerRoutes();
  server.begin();

  running = true;
  if (xTaskCreate(webserverTaskFunc, "webserver", 4096, nullptr, 3, &webserverTask) != pdPASS) {
    running = false;
    webserverTask = nullptr;
    LOGW(TAG, "Webserver-Task konnte nicht erstellt werden");
    return;
  }

  IPAddress apIp = WiFi.softAPIP();
  LOGI(TAG, "Webserver aktiv: AP '%s' (offen) unter %s", AP_SSID, apIp.toString().c_str());
}

void webserverStop() {
  if (!running)
    return;

  running = false;

  // Task kurz auslaufen lassen (er löscht sich selbst).
  if (webserverTask != nullptr) {
    for (int i = 0; i < 20 && eTaskGetState(webserverTask) != eDeleted; i++)
      vTaskDelay(pdMS_TO_TICKS(10));
    if (eTaskGetState(webserverTask) != eDeleted)
      vTaskDelete(webserverTask);
    webserverTask = nullptr;
  }

  server.stop();
  dnsServer.stop();

  WiFi.softAPdisconnect(true);   // AP aus + WLAN kurz stoppen
  WiFi.mode(WIFI_STA);           // zurück auf STA (WLAN wieder starten)

  LOGI(TAG, "Webserver gestoppt");
}

bool webserverIsActive() {
  return running;
}
