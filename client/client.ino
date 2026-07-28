/*
 * rickroll einabauen
 */

#include <WiFi.h>
#include <HardwareSerial.h>
#include <esp_wifi.h>
#include <vector>

// for lwip
#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include <lwip/inet.h>
#include <errno.h>
//
#include "lwip/tcp.h"
#include "lwip/ip_addr.h"

// WLAN-Zugangsdaten
const char *ssid = "GameOfWlan";
const char *password = "thenorthremembers";
const char *ssid2 = "Fairphone 6";
const char *password2 = "Hurensohn";


// Server-Konfiguration
const char *server_address = "morse.hopto.org";  // IP des Servers
const int port = 5100;                           // Port des Servers Senden

// Schräubchen zum drehen
#define SAMPLING_RATE_MS 10  // eine Abtastung alle x ms
#define RECORDING_TIMEOUT_MS 5000
#define TCP_TIMEOUT 30000
#define KEEP_ALIVE_INTERVAL_MS 5000
#define MAX_CONNECT_RETRIES 5
#define QUEUE_SIZE 10
#define SOUND_FREQ 200

#define TX_PIN 17
#define RX_PIN 16
#define SPEAKER 12
#define BUTTON 14
#define LED 13
#define MOSFET 18
#define NORMAL_MODE_PIN 27
#define NO_SOUND_MODE_PIN 26
#define NO_PRINTER_MODE_PIN 25
#define SELF_CHECK_MODE_PIN 33
#define SERVER_CHECK_MODE_PIN 32
#define RICK_ROLL_MODE_PIN 35

enum ConnectionState {
  WIFI_CONNECT,
  DNS_RESOLVE,
  TCP_CONNECT,
  RUNNING
};

struct Package {
  uint8_t status;                // 0 -> keep alive, 1 -> normal package, 2 -> confirm package received, 3 -> server_check_mode (send back)
  uint16_t size;                 // reicht für 87 Minuten
  std::vector<uint8_t> payload;  // dynamische größe
};

ConnectionState state = WIFI_CONNECT;

IPAddress server_ip;

volatile bool NO_SOUND_MODE = false;
volatile bool NO_PRINTER_MODE = false;
volatile bool SELF_CHECK_MODE = false;
volatile bool SERVER_CHECK_MODE = false;
volatile bool RICK_ROLL_MODE = false;

QueueHandle_t sendQueue;      // Kommunikationsschnittstelle von InputTask -> tcpTask
QueueHandle_t playbackQueue;  // Kommunikationsschnittstelle von sortingTask -> outputTask
QueueHandle_t printQueue;     // Kommunikationsschnittstelle von sortingTask -> printTask

int sock = -1;
HardwareSerial printer(2);  // use UART2 (GPIO17 TX, GPIO16 RX)

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

  Serial.printf("Arduino ESP32 Core: %s\n", ESP.getSdkVersion());

  //vTaskDelete(NULL);  // Beendet den Arduino-Loop-Task
}

void loop() {
  vTaskDelay(portMAX_DELAY);  // legt den loop task still
}

/*
 * ingesamt werden alle pins jede Sekunde aktualisiert 
 * und in globalen variablen für die anderen Tasks gespeichert
 * dazwischen wird kurz der MOSFET ausgeschaltet, damit der esp32 bei rückfluss einfach ausgeht
 */
void CheckerTask(void *pvParameters) {
  while (true) {
    testMosfet();  // contains 100ms pause
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

    switch (state) {
      case WIFI_CONNECT:
        showSearchingWiFi();
        break;

      case DNS_RESOLVE:
        //Serial.printf("RSSI: %d dBm\n", WiFi.RSSI());
        showResolvingDNS();
        break;

      case TCP_CONNECT:
        //Serial.printf("RSSI: %d dBm\n", WiFi.RSSI());
        showConnectTCP();
        break;

      case RUNNING:
        //Serial.printf("queues (size:%d): send %d play %d print %d \n", QUEUE_SIZE, uxQueueMessagesWaiting(sendQueue), uxQueueMessagesWaiting(playbackQueue), uxQueueMessagesWaiting(printQueue));
        //Serial.printf("Heap free: %u Min heap: %u \n", ESP.getFreeHeap(), ESP.getMinFreeHeap());
        //char stats[512];
        //vTaskGetRunTimeStats(stats);
        //Serial.printf("stats: %s\n", stats);
        //Serial.printf("RSSI: %d dBm\n", WiFi.RSSI());
        if (WiFi.RSSI() < -70)
          showBadSignal();
        //uint8_t mac[6];
        //WiFi.macAddress(mac);
        //Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        break;
    }
  }
}

void ConnectionTask(void *pvParameters) {
  bool was_connected = false;
  int retry_counter = 0;

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
        server_ip = IPAddress(0, 0, 0, 0);
        Serial.println("Resolving DNS...");
        if (WiFi.hostByName(server_address, server_ip) && server_ip != IPAddress(0, 0, 0, 0)) {
          Serial.printf("Server IP: %s\n", server_ip.toString().c_str());
          state = TCP_CONNECT;
          retry_counter = 0;
        } else {
          retry_counter++;
          Serial.printf("DNS failed or invalid (%d/%d)\n", retry_counter, MAX_CONNECT_RETRIES);
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
        if (connectTCP(retry_counter)) {
          retry_counter = 0;
          state = RUNNING;
        } else {
          retry_counter++;
          vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        if (retry_counter == MAX_CONNECT_RETRIES) {
          retry_counter = 0;
          state = DNS_RESOLVE;
          Serial.println("TCP failed, returning to DNS_RESOLVE");
        }
        break;

      case RUNNING:
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
        }
        break;
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

void InputTask(void *pvParameters) {
  Package package;
  unsigned long last_pressed = millis() - RECORDING_TIMEOUT_MS;

  while (true) {
    // nichts tun solange keine wifi oder server verbindung
    while (state != RUNNING && SELF_CHECK_MODE == false)
      vTaskDelay(10 / portTICK_PERIOD_MS);

    // ein neues Package anfangen
    if (digitalRead(BUTTON) == false) {  // button pressed
      last_pressed = millis();
      // ein packet aufnehmen, bis lange nichts mehr gedrückt
      while (millis() - last_pressed < RECORDING_TIMEOUT_MS) {
        Serial.printf("Time until send: %d \n", RECORDING_TIMEOUT_MS + last_pressed - millis());
        // taste einen frame ab
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
      // fertiges Package
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

void PlaybackTask(void *pvParameters) {
  Package package;
  bool sound_on = false;

  while (true) {
    vTaskDelay(100 / portTICK_PERIOD_MS);  // damit nicht gepollt wird

    // wenn ein/kein package in der leitung ist
    if (getPackageFromQueue(playbackQueue, package) != pdPASS)
      continue;

    // packet abarbeiten
    for (uint8_t signal : package.payload) {
      uint8_t mask = 0b10000000;

      for (int i = 0; i < 8; i++) {
        if ((signal & mask)) {
          // damit es nicht knattert, wenn es ein durchgängiges signal gibt
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

void PrintTask(void *pvParameters) {
  Package package;
  static bool top_line[384] = { false };
  static bool bottom_line[384] = { false };
  int index = 0;
  bool writing_top_line = true;

  while (true) {
    // um polling zu vermeiden
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // wenn ein/kein package in der leitung ist
    if (uxQueueMessagesWaiting(printQueue) == 0)
      continue;

    // package auslesen
    getPackageFromQueue(printQueue, package);

    // überspringen, wenn no printer mode
    if (NO_PRINTER_MODE == true)
      continue;

    // packet abarbeiten
    for (uint8_t signal : package.payload) {
      uint8_t mask = 0b10000000;

      for (int j = 0; j < 8; j++) {
        // ob das in die erste oder zweite zeile des druckers kommt
        if (writing_top_line)
          top_line[index] = (signal & mask);
        else
          bottom_line[index] = (signal & mask);

        index++;

        // wenn eine zeile gedruckt werden kann
        if (index == 384) {
          if (writing_top_line == true) {
            writing_top_line = false;
            index = 0;
          } else {
            print(top_line, bottom_line);
            memset(top_line, 0, sizeof(top_line));
            memset(bottom_line, 0, sizeof(bottom_line));
            writing_top_line = true;
            index = 0;
          }
        }
      }
      mask >>= 1;
    }
    if (index != 0) {
      print(top_line, bottom_line);
      memset(top_line, 0, sizeof(top_line));
      memset(bottom_line, 0, sizeof(bottom_line));
      writing_top_line = true;
      index = 0;
    }
    // Zeilenvorschub
    printer.write('\n');
    printer.write('\n');
    //printer.write('\n');
    printer.flush();
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}