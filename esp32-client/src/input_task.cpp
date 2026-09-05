// =============================================================================
// input_task.cpp  –  "Input Task" (CPU 1, Priorität 1)
//
// Liest die Morse-Taste aus und baut daraus ein Morse-Paket. Dazu wird die
// Taste alle SAMPLING_RATE_MS Millisekunden abgefragt und das Ergebnis als Bit
// gespeichert (1 = gedrückt, 0 = losgelassen). Jeweils 8 Abtastungen ergeben
// ein Byte. Wird RECORDING_TIMEOUT_MS lang nichts gedrückt, gilt die Eingabe
// als fertig und das Paket wird in die passende Queue gelegt.
// =============================================================================

#include "tasks.h"

#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "app_state.h"
#include "config.h"
#include "package.h"

static const char* TAG = "input";

// Aktuelle Zeit in Millisekunden seit Systemstart.
// esp_timer_get_time() liefert MIKROsekunden -> deshalb durch 1000 teilen.
static uint32_t nowMs() {
  return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

void InputTask(void* pvParameters) {
  Package package;
  // Zeitstempel des letzten Tastendrucks. So vorbelegt, dass der erste
  // Tastendruck sofort als "neue Eingabe" erkannt wird.
  uint32_t last_pressed = nowMs() - RECORDING_TIMEOUT_MS;

  while (true) {
    // Nichts tun, solange weder Netzwerk läuft noch der Selbsttest aktiv ist.
    // (Sonst würden Tastendrücke ins Leere laufen.)
    /*if (state != RUNNING && SELF_CHECK_MODE == false) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }*/

    // Kein Tastendruck -> kurz schlafen und weiter pollen.
    // WICHTIG: Ohne diese Pause wäre dies eine Busy-Loop, die den IDLE-Task
    // auf CPU 1 aushungert und damit den Task-Watchdog auslöst.
    if (gpio_get_level((gpio_num_t)BUTTON) != 0) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    // --- Taste ist gedrückt: neues Paket aufnehmen. ---
    {
      last_pressed = nowMs();

      // Aufnehmen, bis RECORDING_TIMEOUT_MS lang nichts mehr gedrückt wurde.
      while (nowMs() - last_pressed < RECORDING_TIMEOUT_MS) {
        // Debug-Ausgabe: Restzeit bis zum automatischen Senden.
        ESP_LOGI(TAG, "Time until send: %u", (unsigned)(RECORDING_TIMEOUT_MS + last_pressed - nowMs()));

        // Einen "Frame" = 8 Abtastungen = 1 Byte aufnehmen.
        uint8_t signal = 0;
        for (int j = 0; j < 8; j++) {
          signal <<= 1;                                     // Platz fürs nächste Bit
          if (gpio_get_level((gpio_num_t)BUTTON) == 0) {    // Taste gedrückt (LOW)?
            signal++;                                        // -> Bit 1 setzen
            last_pressed = nowMs();                          // Zeitstempel erneuern
          }
          vTaskDelay(pdMS_TO_TICKS(SAMPLING_RATE_MS)); // eine Abtast-Periode warten
        }
        package.payload.push_back(signal);  // fertiges Byte anhängen
      }

      // --- Paket ist fertig: passenden Zielort wählen. ---
      package.size = package.payload.size();

      if (SELF_CHECK_MODE) {
        // Selbsttest: direkt an die Wiedergabe schicken (kein Netzwerk).
        if (!putPackageIntoQueue(playbackQueue, package))
          ESP_LOGW(TAG, "playbackQueue overflow");
      } else if (SERVER_CHECK_MODE) {
        // Server-Check: Paket wird zum Server geschickt und kommt zurück.
        package.status = 3;
        if (!putPackageIntoQueue(sendQueue, package))
          ESP_LOGW(TAG, "sendQueue overflow");
      } else {
        // Normalbetrieb: normales Paket an den Server senden.
        package.status = 1;
        if (!putPackageIntoQueue(sendQueue, package))
          ESP_LOGW(TAG, "sendQueue overflow");
      }
      package.payload.clear();  // Speicher fürs nächste Paket freigeben
    }
  }
}

