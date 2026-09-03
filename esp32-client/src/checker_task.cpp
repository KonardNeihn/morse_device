#include "tasks.h"

#include <Arduino.h>
#include <WiFi.h>

#include "app_state.h"
#include "hardware.h"

/*
 * Insgesamt werden alle Pins jede Sekunde aktualisiert und in globalen
 * Variablen für die anderen Tasks gespeichert. Dazwischen wird kurz der
 * MOSFET ausgeschaltet, damit der ESP32 bei Rückstrom einfach ausgeht.
 */
void CheckerTask(void* pvParameters) {
  while (true) {
    testMosfet();  // enthält 100 ms Pause
    checkPins();
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    /*|            RSSI | Qualität      |
      | --------------: | ------------- |
      |       > -50 dBm | ausgezeichnet |
      | -50 bis -60 dBm | sehr gut      |
      | -60 bis -67 dBm | gut           |
      | -67 bis -70 dBm | ausreichend   |
      | -70 bis -80 dBm | schwach       |
      |       < -80 dBm | kritisch      |*/

    // im self check mode keine Statusmeldung über die Lampe
    if (SELF_CHECK_MODE)
      continue;

    switch (state) {
      case WIFI_CONNECT:
        showSearchingWiFi();
        break;

      case WAIT_FOR_IP6:
        showWaitForIP6();
        break;

      case DNS_RESOLVE:
        showResolvingDNS();
        break;

      case TCP_CONNECT:
        showConnectTCP();
        break;

      case RUNNING:
        if (WiFi.RSSI() < -70)
          showBadSignal();
        break;
    }
  }
}
