#include "tasks.h"

#include <Arduino.h>

#include "app_state.h"
#include "config.h"
#include "hardware.h"
#include "package.h"

void PlaybackTask(void* pvParameters) {
  Package package;
  bool sound_on = false;

  while (true) {
    vTaskDelay(100 / portTICK_PERIOD_MS);  // damit nicht gepollt wird

    // wenn ein/kein Paket in der Leitung ist
    if (!getPackageFromQueue(playbackQueue, package))
      continue;

    // Paket abarbeiten
    for (uint8_t signal : package.payload) {
      uint8_t mask = 0b10000000;

      for (int i = 0; i < 8; i++) {
        if ((signal & mask)) {
          // damit es nicht knattert, wenn es ein durchgängiges Signal gibt
          if (sound_on == false) {
            playTone();
            digitalWrite(LED, HIGH);
            sound_on = true;
          }
        } else {
          noTone(SPEAKER);
          digitalWrite(LED, LOW);
          sound_on = false;
        }
        mask >>= 1;
        vTaskDelay(SAMPLING_RATE_MS / portTICK_PERIOD_MS);
      }
    }
    noTone(SPEAKER);
    digitalWrite(LED, LOW);
    sound_on = false;

    if (!putPackageIntoQueue(printQueue, package))
      Serial.println("printQueue overflow");
  }
}
