// =============================================================================
// playback_task.cpp  –  "Output Task" (CPU 1, Priorität 1)
//
// Nimmt Morse-Pakete aus der playbackQueue (vom Netzwerk) und spielt sie ab:
// Für jedes Bit wird der Lautsprecher (Ton) und die LED ein-/ausgeschaltet.
// Ein gesetztes Bit = Ton an, ein klares Bit = Ton aus. Damit es bei
// durchgehenden Signalen nicht "knattert", wird der Ton nur beim Wechsel
// 0 -> 1 neu gestartet (Merker `sound_on`).
// =============================================================================

#include "tasks.h"

#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "app_state.h"
#include "config.h"
#include "hardware.h"
#include "package.h"

static const char* TAG = "playback";

void PlaybackTask(void* pvParameters) {
  Package package;
  bool sound_on = false;  // merkt sich, ob gerade ein Ton läuft

  while (true) {
    vTaskDelay(pdMS_TO_TICKS(100));  // kleine Pause, damit nicht nur gepollt wird

    // Kein Paket in der Queue? -> weiter warten.
    if (!getPackageFromQueue(playbackQueue, package))
      continue;

    // Paket Byte für Byte und Bit für Bit abarbeiten.
    for (uint8_t signal : package.payload) {
      uint8_t mask = 0b10000000;  // mit dem höchstwertigen Bit (MSB) beginnen

      for (int i = 0; i < 8; i++) {
        if ((signal & mask)) {
          // Bit gesetzt = Ton an. Nur neu einschalten, wenn vorher aus war,
          // sonst gibt es bei mehreren Einsen hintereinander ein "Knacken".
          if (sound_on == false) {
            playTone();
            gpio_set_level((gpio_num_t)LED, 1);
            sound_on = true;
          }
        } else {
          // Bit klar = Ton aus.
          stopTone();
          gpio_set_level((gpio_num_t)LED, 0);
          sound_on = false;
        }
        mask >>= 1;
        vTaskDelay(pdMS_TO_TICKS(SAMPLING_RATE_MS));  // Dauer pro Bit
      }
    }
    // Nach dem letzten Bit sicherstellen, dass Ton und LED aus sind.
    stopTone();
    gpio_set_level((gpio_num_t)LED, 0);
    sound_on = false;

    // Paket an den Druck-Task weiterreichen (printQueue).
    if (!putPackageIntoQueue(printQueue, package))
      ESP_LOGW(TAG, "printQueue overflow");
  }
}

