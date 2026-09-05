// =============================================================================
// checker_task.cpp  –  "Checker Task" (CPU 1, Priorität 2)
//
// Liest regelmäßig (ca. jede Sekunde) die Drehschalter ein und schreibt die
// Modus-Flags. Außerdem zeigt er über die Status-LED, in welchem Netzwerk-
// Zustand sich das Gerät gerade befindet (Blinkmuster). Zwischendurch schaltet
// er kurz den MOSFET aus (testMosfet), damit der ESP32 bei Strom-Rückfluss
// sauber ausgeht.
// =============================================================================

#include "tasks.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "app_state.h"
#include "hardware.h"
#include "network.h"

void CheckerTask(void* pvParameters) {
  while (true) {
    testMosfet();  // enthält eine 100-ms-Pause (MOSFET kurz aus)
    checkPins();   // Drehschalter einlesen -> Modus-Flags setzen
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Orientierung zur Signalstärke (RSSI in dBm):
    //   > -50 dBm          ausgezeichnet
    //   -50 bis -60 dBm    sehr gut
    //   -60 bis -67 dBm    gut
    //   -67 bis -70 dBm    ausreichend
    //   -70 bis -80 dBm    schwach
    //   < -80 dBm          kritisch

    // Im Selbsttest-Modus keine Status-LED (das Gerät macht gerade Morse).
    if (SELF_CHECK_MODE)
      continue;

    // Je nach Netzwerk-Zustand ein anderes Blinkmuster zeigen.
    switch (state) {
      case WIFI_CONNECT:
        showStatus(1);  // 1x blinken
        break;

      case WAIT_FOR_IP6:
        showStatus(2);  // 2x blinken
        break;

      case DNS_RESOLVE:
        showStatus(3);  // 3x blinken
        break;

      case TCP_CONNECT:
        showStatus(4);  // 4x blinken
        break;

      case RUNNING:
        if (wifiRssi() < -70) // nur bei schlechtem Signal warnen
          showStatus(5);  // 5x blinken
        break;
    }
  }
}

