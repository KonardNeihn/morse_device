#include "hardware.h"

#include <Arduino.h>

#include "app_state.h"
#include "config.h"

void showSearchingWiFi() {
  for (int i = 0; i < 1; i++) {
    digitalWrite(LED, HIGH);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    digitalWrite(LED, LOW);
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

void showWaitForIP6() {
  for (int i = 0; i < 2; i++) {
    digitalWrite(LED, HIGH);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    digitalWrite(LED, LOW);
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

void showResolvingDNS() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED, HIGH);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    digitalWrite(LED, LOW);
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

void showConnectTCP() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(LED, HIGH);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    digitalWrite(LED, LOW);
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

void showBadSignal() {
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED, HIGH);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    digitalWrite(LED, LOW);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void playTone() {
  if (!NO_SOUND_MODE)           // nur wenn der Drehschalter nicht "ohne Ton" sagt (LOW = angeschaltet)
    tone(SPEAKER, SOUND_FREQ);  // tone() blockiert auf dem ESP32 den Thread NICHT
}

void checkPins() {
  NO_SOUND_MODE = (digitalRead(NO_SOUND_MODE_PIN) == LOW);
  NO_PRINTER_MODE = (digitalRead(NO_PRINTER_MODE_PIN) == LOW);
  SELF_CHECK_MODE = (digitalRead(SELF_CHECK_MODE_PIN) == LOW);
  SERVER_CHECK_MODE = (digitalRead(SERVER_CHECK_MODE_PIN) == LOW);
  RICK_ROLL_MODE = (digitalRead(RICK_ROLL_MODE_PIN) == LOW);
}

void testMosfet() {
  digitalWrite(MOSFET, LOW);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  digitalWrite(MOSFET, HIGH);
}
