// =============================================================================
// checker_task.cpp  –  "Checker Task" (Priorität 2)
//
// Liest regelmäßig (ca. jede Sekunde) die Drehschalter ein und schreibt die
// Modus-Flags. Außerdem zeigt er über die Status-LED, in welchem Netzwerk-
// Zustand sich das Gerät gerade befindet (Blinkmuster). Zwischendurch schaltet
// er kurz den MOSFET aus (testMosfet), damit der ESP32 bei Strom-Rückfluss
// sauber ausgeht.
// =============================================================================

#include "app.h"
#include "freertos/projdefs.h"
#include "hardware.h"
#include "log.h"
#include "network.h"

#include <freertos/task.h>          // vTaskDelay, pdMS_TO_TICKS

static const char* TAG = "checker";

void CheckerTask(void* pvParameters) {
  while (true) {
    testMosfet();  // enthält eine 100-ms-Pause (MOSFET kurz aus)
    checkPins();   // Kippschalter einlesen -> Modus-Flags setzen
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Orientierung zur Signalstärke (RSSI in dBm):
    //   > -50 dBm          ausgezeichnet
    //   -50 bis -60 dBm    sehr gut
    //   -60 bis -67 dBm    gut
    //   -67 bis -70 dBm    ausreichend
    //   -70 bis -80 dBm    schwach
    //   < -80 dBm          kritisch

    // nicht blinken, wenn led schon benutzt wird
    if (isRecording || isPlaying) {
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    // Je nach Netzwerk-Zustand ein anderes Blinkmuster zeigen.
    switch (state) {
      case WIFI_CONNECT:
        showStatus(1);  // 1x blinken
        break;

      case TCP_CONNECT:
        showStatus(2);  // 2x blinken
        break;

      case RUNNING:
        if (wifiRssi() < -70) { // nur bei schlechtem Signal warnen
          showStatus(3); // 3x blinken
          LOGW(TAG, "bad signal! rssi: %d dB", wifiRssi());
        } 
        break;
    }
  }
}

