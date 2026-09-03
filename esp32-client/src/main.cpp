#include <Arduino.h>
#include <WiFi.h>

#include "app_state.h"
#include "config.h"
#include "network.h"
#include "package.h"
#include "tasks.h"

void setup() {
  pinMode(SPEAKER, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(MOSFET, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(NORMAL_MODE_PIN, INPUT);
  pinMode(NO_SOUND_MODE_PIN, INPUT);
  pinMode(NO_PRINTER_MODE_PIN, INPUT);
  pinMode(SELF_CHECK_MODE_PIN, INPUT);
  pinMode(SERVER_CHECK_MODE_PIN, INPUT);
  pinMode(RICK_ROLL_MODE_PIN, INPUT);

  Serial.begin(115200);
  printer.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.printf("ESP32 Arduino Core: %s\n", ESP.getSdkVersion());

  sendQueue = xQueueCreate(QUEUE_SIZE, sizeof(Package *));
  playbackQueue = xQueueCreate(QUEUE_SIZE, sizeof(Package *));
  printQueue = xQueueCreate(QUEUE_SIZE, sizeof(Package *));

  WiFi.onEvent([](arduino_event_id_t event, arduino_event_info_t info) {
    Serial.printf("Event: %s (%d)\n", wifiEventName(event), event);

    if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
      auto reason = info.wifi_sta_disconnected.reason;
      Serial.printf("Disconnected: %s (%d)\n", disconnectReason(reason), reason);
    }
  });

  xTaskCreatePinnedToCore(ConnectionTask, "Check WiFi TCP", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(InputTask, "Input Task", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(PlaybackTask, "Output Task", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(CheckerTask, "Checker Task", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(PrintTask, "Print Task", 4096, NULL, 2, NULL, 1);

  // vTaskDelete(NULL);  // Beendet den Arduino-Loop-Task
}

void loop() {
  vTaskDelay(portMAX_DELAY);  // legt den Loop-Task still
}

