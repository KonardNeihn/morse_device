// =============================================================================
// connection_task.cpp  –  "Check WiFi TCP" (läuft auf CPU 0, Priorität 1)
//
// Das ist die Netzwerk-Zustandsmaschine des Geräts. Der Task arbeitet einen
// Kreislauf aus vier Zuständen ab:
//
//   WIFI_CONNECT  ->  WLAN suchen/verbinden (erst ssid, dann ssid2)
//   DNS_RESOLVE   ->  Hostname des Servers in eine IPv6-Adresse auflösen
//   TCP_CONNECT   ->  TCP-Verbindung zum Server aufbauen
//   RUNNING       ->  Dauerbetrieb: eingehende Morse-Pakete empfangen und
//                     wartende Pakete an den Server senden
//
// Bei Fehlern springt er immer wieder einen Schritt zurück (mit begrenzten
// Wiederholungen), bei hartnäckigen WLAN-Problemen startet er den ESP neu.
// =============================================================================

#include "tasks.h"

#include <esp_log.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "app_state.h"
#include "config.h"
#include "network.h"

static const char* TAG = "connection";

void ConnectionTask(void* pvParameters) {
  // Merkt sich, ob schon einmal eine WLAN-Verbindung bestand. Nur dann wird
  // im WIFI_CONNECT-Zustand zuerst ein "Reconnect" statt eines Neustarts versucht.
  bool was_connected = false;
  // Zählt fehlgeschlagene DNS-/TCP-Versuche. Nach MAX_CONNECT_RETRIES wird
  // ein Zustand zurückgesprungen (z. B. TCP -> DNS -> WLAN).
  int retry_counter = 0;

  // Kurz warten, damit der CheckerTask zuerst die Drehschalter einlesen und
  // die Modus-Flags (z. B. SELF_CHECK_MODE) setzen kann.
  vTaskDelay(pdMS_TO_TICKS(500));

  while (true) {
    // Selbsttest-Modus: Kein Netzwerk! Das Gerät prüft sich nur selbst
    // (Taste -> Morse -> Lautsprecher). Deshalb hier einfach nichts tun.
    if (SELF_CHECK_MODE) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    switch (state) {
      case WIFI_CONNECT:
        // Schon mal verbunden gewesen? Dann zuerst versuchen, die alte
        // Verbindung wiederherzustellen, bevor neu gestartet wird.
        if (was_connected) {
          vTaskDelay(pdMS_TO_TICKS(1000));
          if (wifiIsConnected()) {
            state = WAIT_FOR_IP6;  // erst IPv6 abwarten, dann DNS/TCP
            ESP_LOGI(TAG, "WiFi reconnected alone");
            break;
          }

          ESP_LOGI(TAG, "reconnecting Wifi...");
          esp_wifi_connect();  // Verbindungsaufbau zum konfigurierten AP erneut anstoßen
          for (int i = 0; i < 200 && !wifiIsConnected(); i++)
            vTaskDelay(pdMS_TO_TICKS(50));

          if (wifiIsConnected()) {
            state = WAIT_FOR_IP6;  // erst IPv6 abwarten, dann DNS/TCP
            ESP_LOGI(TAG, "WiFi reconnected");
            break;
          } else {
            ESP_LOGI(TAG, "couldnt reconnect wifi");
          }
        }

        // Erste WLAN-Verbindung: primäres Netzwerk versuchen.
        if (connectWifi(ssid, password)) {
          state = DNS_RESOLVE;
          was_connected = true;
          break;
        }

        // Wenn das primäre Netz fehlschlägt: zweites Netzwerk als Fallback.
        if (connectWifi(ssid2, password2)) {
          state = DNS_RESOLVE;
          was_connected = true;
          break;
        }

        // Beide Netze nicht erreichbar -> Neustart (hilft bei WLAN oft).
        ESP_LOGW(TAG, "Restarting...");
        esp_restart();  // ESP32 komplett neu starten (wie Reset-Knopf)
        break;

      case WAIT_FOR_IP6:
        // Nach einer WLAN-(Re-)Verbindung auf die globale IPv6-Adresse warten.
        // Ohne sie ist der IPv6-Server nicht erreichbar ("Host is unreachable",
        // Fehler 118). Erst danach DNS auflösen und TCP verbinden.
        if (waitForIPv6Address()) {
          state = DNS_RESOLVE;
        } else {
          // Kein IPv6 bekommen -> WLAN vollständig neu aufbauen.
          ESP_LOGW(TAG, "No IPv6, full WiFi reconnect");
          esp_wifi_disconnect();  // Verbindung trennen, damit der Neuaufbau sauber startet
          state = WIFI_CONNECT;
        }
        break;

      case DNS_RESOLVE:
        // Ohne WLAN gibt es auch kein DNS -> zurück zum WLAN.
        if (!wifiIsConnected()) {
          state = WIFI_CONNECT;
          break;
        }

        if (resolveDNS()) {
          state = TCP_CONNECT;
          retry_counter = 0;
        } else {
          retry_counter++;
          vTaskDelay(pdMS_TO_TICKS(50));
        }

        // Zu viele Fehlversuche -> nochmal von vorn mit WLAN.
        if (retry_counter == MAX_CONNECT_RETRIES) {
          retry_counter = 0;
          state = WIFI_CONNECT;
          ESP_LOGW(TAG, "DNS failed, returning to WIFI_CONNECT");
        }
        break;

      case TCP_CONNECT:
        if (!wifiIsConnected()) {
          state = WIFI_CONNECT;
          break;
        }
        retry_counter++;
        if (connectTCP(retry_counter)) {
          retry_counter = 0;
          state = RUNNING;
        } else {
          disconnectTCP();
          vTaskDelay(pdMS_TO_TICKS(1000));
        }

        if (retry_counter == MAX_CONNECT_RETRIES) {
          retry_counter = 0;
          state = WIFI_CONNECT;
          ESP_LOGW(TAG, "TCP failed, returning to WIFI_CONNECT");
        }
        break;

      case RUNNING: {
        // Verbindung verloren? -> zurück zum WLAN.
        if (!wifiIsConnected()) {
          state = WIFI_CONNECT;
          break;
        }
        // Nicht blockierend prüfen, ob eingehende Daten vorhanden sind.
        bool readable;
        if (checkSocket(readable)) {
          // Daten vom Server da? -> Paket empfangen und an die Wiedergabe
          // (playbackQueue -> PlaybackTask) weiterreichen.
          if (readable)
            receivePackage();
          // Versuchen, wartende Pakete (von InputTask) zu senden.
          // sendPackage() prüft selbst, ob etwas in der sendQueue liegt.
          sendPackage();
        } else {
          // Socket defekt -> aufräumen und neu verbinden.
          disconnectTCP();
          state = TCP_CONNECT;
        }
        break;
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
