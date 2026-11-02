// =============================================================================
// connection_task.cpp  –  "Check WiFi TCP" (Priorität 1)
//
// Das ist die Netzwerk-Zustandsmaschine des Geräts. Der Task arbeitet einen
// Kreislauf aus drei Zuständen ab:
//
//   WIFI_CONNECT  ->  WLAN verbinden (WiFiMulti probiert alle konfigurierten APs)
//   TCP_CONNECT   ->  TCP-Verbindung zum Server aufbauen
//   RUNNING       ->  Dauerbetrieb: eingehende Morse-Pakete empfangen und
//                     wartende Pakete an den Server senden
//
// Bei Fehlern springt er immer wieder einen Schritt zurück.
// =============================================================================

#include "app.h"
#include "config.h"
#include "network.h"
#include "storage.h"
#include "log.h"
#include "webserver.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "connection";

// Zählt fehlgeschlagene DNS-/TCP-Versuche. Nach MAX_CONNECT_RETRIES wird
// ein Zustand zurückgesprungen (z. B. TCP -> DNS -> WLAN).
static int retry_counter = 0;

static bool hotspot_mode_was_active = false;

static const char* stateName(ConnectionState s) {
  switch (s) {
    case WIFI_CONNECT: return "WIFI_CONNECT";
    case TCP_CONNECT:  return "TCP_CONNECT";
    case RUNNING:      return "RUNNING";
  }
  return "?";
}
static void setState(ConnectionState s) {
  if (s != state) {
    LOGI(TAG, "state: %s -> %s", stateName(state), stateName(s));
    state = s;
  }
}

void ConnectionTask(void *pvParameters) {

  // Kurz warten, damit der CheckerTask zuerst die Kippschalter lesen kann.
  vTaskDelay(pdMS_TO_TICKS(500));

  while (true) {

    // Hotspot-Modus: AP + Webserver laufen PARALLEL zum normalen Betrieb.
    if (HOTSPOT_MODE && !hotspot_mode_was_active) {
      webserverStart();
      hotspot_mode_was_active = true;
    } else if (!HOTSPOT_MODE && hotspot_mode_was_active) {
      webserverStop();
      hotspot_mode_was_active = false;
    }

    // Neue WLAN-Daten wurden im Portal gespeichert -> sauber neu verbinden.
    if (newWifiCredentials) {
      newWifiCredentials = false;
      disconnectTCP();
      configureWifiCredentials();
      state = WIFI_CONNECT;
    }

    if (!tcpIsConnected() && state == RUNNING)
      setState(TCP_CONNECT);

    // Verbindung verloren? -> zurück zum WLAN.
    if (!wifiIsConnected())
      setState(WIFI_CONNECT);



    switch (state) {
      case WIFI_CONNECT:

        if (wifiIsConnected()){
          setState(TCP_CONNECT);
          break;
        }
        connectWifi();
        vTaskDelay(pdMS_TO_TICKS(1000));

        break;

      case TCP_CONNECT:

        // try to connect via TCP
        if (connectTCP()) {
          retry_counter = 0;
          setState(RUNNING);
          // Nach jedem Verbinden mit eigener MAC-Adresse registrieren
          sendRegister();
          // Ausstehende, noch nicht bestätigte Nachrichten erneut senden
          resendPendingSends();
        } else {
          retry_counter++;
          LOGI(TAG, "couldn't connect TCP #%d", retry_counter);
          disconnectTCP();
        }

        if (retry_counter == MAX_CONNECT_RETRIES) {
          retry_counter = 0;
          setState(WIFI_CONNECT);
          LOGW(TAG, "TCP failed, returning to WIFI_CONNECT");
        }
        break;

      case RUNNING: {

        // TCP-Verbindung zum Server defekt?
        if (!tcpIsConnected()) {
          setState(TCP_CONNECT);
          break;
        }

        // Nur empfangen, wenn a) Daten da sind UND b) Platz in der
        // Wiedergabe-Queue
        if (tcpAvailableMessage() && uxQueueMessagesWaiting(playbackQueue) < QUEUE_SIZE)
          receivePackage();

        // Nur senden, wenn überhaupt etwas in der Send-Queue liegt
        if (uxQueueMessagesWaiting(sendQueue) > 0)
          sendPackage();
      }
    }

    // Kurze Pause, damit auch andere Tasks (und vor allem der IDLE-Task!)
    // CPU-Zeit bekommen. Achtung: pdMS_TO_TICKS() RUNDET AB (trunkiert) –
    // bei 100 Hz Tick-Rate ergeben Werte unter 10 ms 0 Ticks (= keine Pause).
    // Deshalb hier 10 ms verwenden, damit garantiert mindestens 1 Tick gewartet
    // wird und diese Schleife nicht zur Busy-Loop (Task-Watchdog) wird.
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
