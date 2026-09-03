#include "tasks.h"

#include <Arduino.h>

#include "app_state.h"
#include "config.h"
#include "package.h"

void InputTask(void* pvParameters) {
  Package package;
  unsigned long last_pressed = millis() - RECORDING_TIMEOUT_MS;

  while (true) {
    // nichts tun, solange keine WiFi- oder Serververbindung besteht
    while (state != RUNNING && SELF_CHECK_MODE == false)
      vTaskDelay(10 / portTICK_PERIOD_MS);

    // ein neues Paket anfangen
    if (digitalRead(BUTTON) == false) {  // button pressed
      last_pressed = millis();
      // ein Paket aufnehmen, bis lange nichts mehr gedrückt wurde
      while (millis() - last_pressed < RECORDING_TIMEOUT_MS) {
        Serial.printf("Time until send: %d \n", RECORDING_TIMEOUT_MS + last_pressed - millis());
        // taste einen Frame ab
        uint8_t signal = 0;
        for (int j = 0; j < 8; j++) {
          signal <<= 1;
          if (digitalRead(BUTTON) == false) {  // button pressed
            signal++;
            last_pressed = millis();
          }
          vTaskDelay(SAMPLING_RATE_MS / portTICK_PERIOD_MS);
        }
        package.payload.push_back(signal);
      }
      // fertiges Paket
      package.size = package.payload.size();
      if (SELF_CHECK_MODE) {
        if (!putPackageIntoQueue(playbackQueue, package))
          Serial.println("playbackQueue overflow");
      } else if (SERVER_CHECK_MODE) {
        package.status = 3;
        if (!putPackageIntoQueue(sendQueue, package))
          Serial.println("sendQueue overflow");
      } else {
        package.status = 1;
        if (!putPackageIntoQueue(sendQueue, package))
          Serial.println("sendQueue overflow");
      }
      package.payload.clear();
    }
  }
}
