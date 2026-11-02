#include "app.h"
#include "config.h"
#include "esp32-hal.h"
#include "hardware.h"
#include "package.h"
#include "log.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>


// =============================================================================
// input_task.cpp  –  "Input Task" (Priorität 1)
//
// Liest die Morse-Taste aus und baut daraus ein Morse-Paket. Dazu wird die
// Taste alle SAMPLING_RATE_MS Millisekunden abgefragt und das Ergebnis als Bit
// gespeichert (1 = gedrückt, 0 = losgelassen). Jeweils 8 Abtastungen ergeben
// ein Byte. Wird RECORDING_TIMEOUT_MS lang nichts gedrückt, gilt die Eingabe
// als fertig und das Paket wird in die passende Queue gelegt.
// =============================================================================


static const char* TAG = "input";

void InputTask(void* pvParameters) {
  Package package;
  // Zeitstempel des letzten Tastendrucks. So vorbelegt, dass der erste
  // Tastendruck sofort als "neue Eingabe" erkannt wird.
  uint32_t last_pressed = millis() - RECORDING_TIMEOUT_MS;

  while (true) {

    // --- Taste ist gedrückt: neues Paket aufnehmen. ---
    if (buttonIsPressed()) {
      LOGI(TAG, "Nimmt auf...");

      last_pressed = millis();
      isRecording = true;  // PlaybackTask soll während der Aufnahme nichts abspielen und status nicht blinken

      // Aufnehmen, bis RECORDING_TIMEOUT_MS lang nichts mehr gedrückt wurde.
      while (millis() - last_pressed < RECORDING_TIMEOUT_MS) {
        // Einen "Frame" = 8 Abtastungen = 1 Byte aufnehmen.
        uint8_t signal = 0;
        for (int j = 0; j < 8; j++) {
          signal <<= 1;                                     // Platz fürs nächste Bit
          if (buttonIsPressed()) {
            signal++;                                        // -> Bit 1 setzen
            last_pressed = millis();                          // Zeitstempel erneuern
            setLedHigh();               // LED blinkt mit der Morse-Eingabe
          } else {
            setLedLow();              // Taste los -> LED aus
          }
          vTaskDelay(pdMS_TO_TICKS(SAMPLING_RATE_MS)); // eine Abtast-Periode warten
        }
        package.payload.push_back(signal);  // fertiges Byte anhängen
      }
      isRecording = false;                        // Aufnahme beendet
      setLedLow();         // LED aus

      // --- Paket ist fertig: passenden Zielort wählen. ---
      package.size = package.payload.size();
      LOGI(TAG, "Aufnahme fertig: %u Bytes", (unsigned)package.size);

      if (TEST_MODE) {
        // Server-Check: Paket wird zum Server geschickt und kommt zurück.
        package.status = 3;
        if (!putPackageIntoQueue(sendQueue, package))
          LOGW(TAG, "sendQueue overflow");
      } else {
        // Normalbetrieb: normales Paket an den Server senden.
        package.status = 1;
        if (!putPackageIntoQueue(sendQueue, package))
          LOGW(TAG, "sendQueue overflow");
      }
      package.payload.clear();  // Speicher fürs nächste Paket freigeben
    }


    // Kein Tastendruck -> kurz schlafen und weiter pollen.
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
  }
}

