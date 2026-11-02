#include "app.h"
#include "config.h"
#include "hardware.h"
#include "log.h"
#include "package.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// =============================================================================
// playback_task.cpp  –  "Output Task" (Priorität 1)
//
// Nimmt Morse-Pakete aus der playbackQueue (vom Netzwerk) und spielt sie ab:
// Für jedes Bit wird der Lautsprecher (Ton) und die LED ein-/ausgeschaltet.
// Ein gesetztes Bit = Ton an, ein klares Bit = Ton aus. Damit es bei
// durchgehenden Signalen nicht "knattert", wird der Ton nur beim Wechsel
// 0 -> 1 neu gestartet (Merker `sound_on`).
// =============================================================================

static const char* TAG = "playback";

void PlaybackTask(void* pvParameters) {
  Package package;

  while (true) {
    vTaskDelay(pdMS_TO_TICKS(100));  // kleine Pause, damit nicht nur gepollt wird

    // Während der Nutzer selbst morst, keine Pakete abspielen.
    if (isRecording)
      continue;

    // Kein Paket da? -> weiter warten.
    if (uxQueueMessagesWaiting(playbackQueue) == 0)
      continue;

    // Paket auslesen.
    getPackageFromQueue(playbackQueue, package);

    LOGI(TAG, "startet playing package %llu", package.msg_id);

    isPlaying = true;  // Status-LED-Blinken während der Wiedergabe unterdrücken

    // Paket Byte für Byte und Bit für Bit abarbeiten.
    for (uint8_t signal : package.payload) {
      uint8_t mask = 0b10000000;  // mit dem höchstwertigen Bit (MSB) beginnen

      for (int i = 0; i < 8; i++) {

        if ((signal & mask)) {
          playTone();
          setLedHigh();
        } else {
          stopTone();
          setLedLow();
        }
        mask >>= 1;
        vTaskDelay(pdMS_TO_TICKS(SAMPLING_RATE_MS));  // Dauer pro Bit
      }
    }
    isPlaying = false;  // Wiedergabe beendet (auch bei Abbruch)

    // Nach dem letzten Bit sicherstellen, dass Ton und LED aus sind.
    stopTone();
    setLedLow();

    LOGI(TAG, "stopped playing package");

    // Paket an den Druck-Task weiterreichen (printQueue).
    if (!putPackageIntoQueue(printQueue, package))
      LOGW(TAG, "printQueue overflow");
  }
}