#include "tasks.h"

#include <Arduino.h>
#include <WiFi.h>

#include "app_state.h"
#include "config.h"
#include "network.h"

void ConnectionTask(void* pvParameters) {
  bool was_connected = false;
  int retry_counter = 0;

  // damit die Pins erst gecheckt werden können
  vTaskDelay(500 / portTICK_PERIOD_MS);

  while (true) {
    if (SELF_CHECK_MODE) {
      vTaskDelay(10 / portTICK_PERIOD_MS);
      continue;
    }

    switch (state) {
      case WIFI_CONNECT:
        if (was_connected) {
          vTaskDelay(1000 / portTICK_PERIOD_MS);
          if (WiFi.status() == WL_CONNECTED) {
            state = DNS_RESOLVE;
            Serial.println("WiFi reconnected alone");
            break;
          }

          Serial.println("reconnecting Wifi...");
          WiFi.reconnect();  // statt WiFi OFF/ON
          if (WiFi.waitForConnectResult() == WL_CONNECTED) {
            state = DNS_RESOLVE;
            Serial.println("WiFi reconnected");
            break;
          } else {
            Serial.println("couldnt reconnect wifi");
          }
        }

        // connect to first wifi
        if (connectWifi(ssid, password)) {
          state = DNS_RESOLVE;
          was_connected = true;
          break;
        }

        // if connecting to ssid2 failed
        if (connectWifi(ssid2, password2)) {
          state = DNS_RESOLVE;
          was_connected = true;
          break;
        }

        // a good restart often helps reconnecting somehow
        Serial.printf("Restarting...\n");
        ESP.restart();
        break;

      case DNS_RESOLVE:
        if (WiFi.status() != WL_CONNECTED) {
          state = WIFI_CONNECT;
          break;
        }

        if (resolveDNS()) {
          state = TCP_CONNECT;
          retry_counter = 0;
        } else {
          retry_counter++;
          vTaskDelay(50 / portTICK_PERIOD_MS);
        }

        if (retry_counter == MAX_CONNECT_RETRIES) {
          retry_counter = 0;
          state = WIFI_CONNECT;
          Serial.printf("DNS failed, returning to WIFI_CONNECT\n");
        }
        break;

      case TCP_CONNECT:
        if (WiFi.status() != WL_CONNECTED) {
          state = WIFI_CONNECT;
          break;
        }
        retry_counter++;
        if (connectTCP(retry_counter)) {
          retry_counter = 0;
          state = RUNNING;
        } else {
          disconnectTCP();
          vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        if (retry_counter == MAX_CONNECT_RETRIES) {
          retry_counter = 0;
          state = DNS_RESOLVE;
          Serial.println("TCP failed, returning to DNS_RESOLVE");
        }
        break;

      case RUNNING: {
        if (WiFi.status() != WL_CONNECTED) {
          state = WIFI_CONNECT;
          break;
        }
        bool readable, writable;
        if (checkSocket(readable, writable)) {
          if (readable)
            receivePackage();
          if (writable)
            sendPackage();
        } else {
          disconnectTCP();
          state = TCP_CONNECT;
        }
        break;
      }
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}
