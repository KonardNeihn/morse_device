/*
 * server soll bei ctrl c alle threads schließen
 * manchmal startet die playback queue nicht, also arbeitet nicht ab
 * rickroll einabauen
 * wenn self/server check mode, muss fremde packete ignorieren, sonst buffer overflow
 * FEHLER: Verbinde mit WGlanE (65650) wifi:sta is connecting, cannot set config
 */

#include <WiFi.h>
#include <HardwareSerial.h>
#include <esp_wifi.h>
#include <vector>

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

#define INACTIVITY_TIMEOUT_MS 5000
#define QUEUE_SIZE INACTIVITY_TIMEOUT_MS / SAMPLING_RATE_MS
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

enum ReceiveState {
  WAIT_FOR_HEADER,
  WAIT_FOR_PAYLOAD
};

ConnectionState state = WIFI_CONNECT;

IPAddress server_ip;

//bool BUTTON_PRESSED = false;  // muss öfter gecheckt werdem, darum lieber im sample send task
volatile bool NO_SOUND_MODE = false;
volatile bool NO_PRINTER_MODE = false;
volatile bool SELF_CHECK_MODE = false;
volatile bool SERVER_CHECK_MODE = false;
volatile bool RICK_ROLL_MODE = false;

QueueHandle_t sendQueue;      // Kommunikationsschnittstelle von InputTask -> tcpTask
QueueHandle_t playbackQueue;  // Kommunikationsschnittstelle von sortingTask -> outputTask
QueueHandle_t printQueue;     // Kommunikationsschnittstelle von sortingTask -> printTask

unsigned long last_own_activity = millis() - INACTIVITY_TIMEOUT_MS;
ReceiveState receiveState = WAIT_FOR_HEADER;
std::vector<uint8_t> outgoing_signal;
uint16_t bytes_to_read = 0; // reicht für 87 Minuten

WiFiClient client;
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

  sendQueue = xQueueCreate(QUEUE_SIZE, sizeof(uint8_t));
  playbackQueue = xQueueCreate(QUEUE_SIZE, sizeof(uint8_t));
  printQueue = xQueueCreate(QUEUE_SIZE, sizeof(uint8_t));

  xTaskCreatePinnedToCore(ConnectionTask, "Check WiFi TCP", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(InputTask, "Input Task", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(PlaybackTask, "Output Task", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(CheckerTask, "Checker Task", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(PrintTask, "Print Task", 4096, NULL, 2, NULL, 1);

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
  
    switch (state) {
      case WIFI_CONNECT:
        showSearchingWiFi();
        break;
      
      case DNS_RESOLVE:
        showResolvingDNS();
        break;

      case TCP_CONNECT:
        showConnectTCP();
        break;

      case RUNNING:
        Serial.printf("queues (size:%d): send %d play %d print %d \n", QUEUE_SIZE, uxQueueMessagesWaiting(sendQueue), uxQueueMessagesWaiting(playbackQueue), uxQueueMessagesWaiting(printQueue));
        //Serial.printf("Heap free: %u Min heap: %u \n", ESP.getFreeHeap(), ESP.getMinFreeHeap());
        //char stats[512];
        //vTaskGetRunTimeStats(stats);
        //Serial.printf("stats: %s", stats);
        break;
    }
  }
}

void ConnectionTask(void *pvParameters) {
  int wichWiFi = 0;
  static int lost_count = 0;

  while (true) {
    if (SELF_CHECK_MODE) {
      vTaskDelay(10 / portTICK_PERIOD_MS);
      continue;
    }

    switch (state) {

      case WIFI_CONNECT:
        Serial.printf("Connecting to ");
        if (wichWiFi == 0)
          Serial.println(ssid);
        else
          Serial.println(ssid2);

        WiFi.disconnect(true, true);
        WiFi.mode(WIFI_OFF);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        WiFi.mode(WIFI_STA);

        WiFi.setSleep(false);
        esp_wifi_set_ps(WIFI_PS_NONE);
        if (wichWiFi == 0)
          WiFi.begin(ssid, password);
        else
          WiFi.begin(ssid2, password2);

        if (WiFi.waitForConnectResult() == WL_CONNECTED) {
          state = DNS_RESOLVE;
          Serial.println("WiFi OK");
        } else {
          Serial.println("WiFi failed");
          if (wichWiFi == 0)
            wichWiFi = 1;
          else
            wichWiFi = 0;
          vTaskDelay(2000 / portTICK_PERIOD_MS);
        }

        break;

      case DNS_RESOLVE:
        if (WiFi.status() != WL_CONNECTED) {
          Serial.println("WiFi lost");
          state = WIFI_CONNECT;
          break;
        }

        Serial.println("Resolving DNS");
        if (WiFi.hostByName(server_address, server_ip)) {
          Serial.printf("Server IP: %s\n", server_ip.toString().c_str());
          state = TCP_CONNECT;
        } else {
          Serial.println("DNS failed");
          state = WIFI_CONNECT;
        }

        break;

      case TCP_CONNECT:
        if (WiFi.status() != WL_CONNECTED) {
          Serial.println("WiFi lost");
          state = WIFI_CONNECT;
          break;
        }

        if (state == TCP_CONNECT) {
          Serial.println("Connecting TCP");
          client.stop();
          if (client.connect(server_ip, port)) {
            //client.setNoDelay(true);
            //client.setTimeout(5);  // z.B. 5ms
            state = RUNNING;
            // gibts leider nicht client.setKeepAlive(30); // Aktiviere TCP Keep-Alive (falls unterstützt) Sende alle 30 Sekunden ein Keep-Alive-Paket
            Serial.println("TCP connected");
          } else {
            Serial.println("TCP failed");
            vTaskDelay(2000 / portTICK_PERIOD_MS);
          }
        }
        break;

      case RUNNING:
        if (WiFi.status() != WL_CONNECTED) {
          Serial.println("WiFi lost");
          state = WIFI_CONNECT;
          break;
        }

        static int lost_count = 0;
        if (!client.connected()) {
          lost_count++;
          if (lost_count > 3) {  // 3 mal hintereinander
            Serial.println("TCP lost, reconnecting...");
            state = TCP_CONNECT;
            lost_count = 0;
          }
        } else {
          lost_count = 0;
        }

        handlePackets();
        break;
    }
    vTaskDelay(3 / portTICK_PERIOD_MS);
  }
}

void InputTask(void *pvParameters) {
  uint8_t signal;

  while (true) {
    // nichts tun solange keine wifi oder server verbindung
    while (state != RUNNING && SELF_CHECK_MODE == false)
      vTaskDelay(10 / portTICK_PERIOD_MS);

    // taste einen frame ab
    signal = 0;
    for (int j = 0; j < 8; j++) {
      signal <<= 1;
      if (digitalRead(BUTTON) == false)  // button pressed
        signal++;
      vTaskDelay(SAMPLING_RATE_MS / portTICK_PERIOD_MS);
    }

    if (signal != 0)
      last_own_activity = millis();

    if (millis() - last_own_activity > INACTIVITY_TIMEOUT_MS)
      continue;

    // sende den Frame
    if (SELF_CHECK_MODE) {
      if (xQueueSend(printQueue, &signal, 0) != pdPASS)
        Serial.printf("printQueue overflow!!!\n");
      if (xQueueSend(playbackQueue, &signal, 0) != pdPASS)
        Serial.printf("playbackQueue overflow!!!\n");

    } else {
      if (xQueueSend(sendQueue, &signal, 0) != pdPASS)
        Serial.printf("sendQueue overflow!!!\n");
    }
  }
}

void PlaybackTask(void *pvParameters) {
  uint8_t signal;
  bool sound_on = false;

  while (true) {
    if (state == RUNNING) {
      // wenn samples in der leitung sind
      if (xQueueReceive(playbackQueue, &signal, SAMPLING_RATE_MS) == pdPASS) {  // wartet bis neues Element kommt. blockiert die cpu nicht

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
      } else {
        noTone(SPEAKER);
        digitalWrite(LED, LOW);
        sound_on = false;
      }
    } else {
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
  }
}

void PrintTask(void *pvParameters) {
  unsigned long last_received = millis() - INACTIVITY_TIMEOUT_MS;
  uint8_t signal;
  static bool top_line[384] = {false};
  static bool bottom_line[384] = {false};
  int index = 0;
  bool writing_top_line = true;
  bool something_in_it = false;

  while (true) {
    if (xQueueReceive(printQueue, &signal, 10) == pdPASS) {  // guckt, ob neues element da ist
      last_received = millis();
      uint8_t mask = 0b10000000;

      for (int j = 0; j < 8; j++) {

        int width_in_pixels = 1;  // SAMPLING_RATE_MS / 20 einfügen für alle 20 millisec bedeuten ein pixel druck. ohne rest

        // nichts anfangen, nichts zu drucken
        if ((signal & mask) == false && something_in_it == false)
          width_in_pixels = 0;
        else
          something_in_it = true;

        // erstellt und sammelt die daten für den druck
        while (width_in_pixels > 0) {
        
          // ob das in die erste oder zweite zeile des druckers kommt
          if (writing_top_line)
            top_line[index] = (signal & mask);
          else
            bottom_line[index] = (signal & mask);

          index++;
          width_in_pixels--;

          // 
          if (index == 384) {
            if (writing_top_line == true) {
              writing_top_line = false;
              index = 0;
            } else {
              if (NO_PRINTER_MODE == false)
                print(top_line, bottom_line);

              something_in_it = false;
              writing_top_line = true;
              index = 0;
            }
          }
        }
        mask >>= 1;
      }
    } else if (millis() - last_received > INACTIVITY_TIMEOUT_MS && something_in_it == true) {
      if (NO_PRINTER_MODE == false)
        print(top_line, bottom_line);
      something_in_it = false;
      writing_top_line = true;
      index = 0;
    }
  }
}