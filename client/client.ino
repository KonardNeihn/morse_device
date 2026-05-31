/*
 * es könnten noch sachen in drucker und fenster stecken bleiben, wenn stream aufhört
 * rickroll einabauen
 * wenn self/server check mode, muss fremde packete ignorieren, sonst buffer overflow
 * FEHLER: Verbinde mit WGlanE (65650) wifi:sta is connecting, cannot set config
 */

#include <WiFi.h>
#include <HardwareSerial.h>
#include <esp_wifi.h>

// WLAN-Zugangsdaten
const char *ssid2 = "WGlan";
const char *password2 = "51565735623896715310";
const char *ssid = "LestMehrBuchen!";
const char *password = "fluessigesHelium-268,8)";


// Server-Konfiguration
const char *server_address = "morse.hopto.org";  // IP des Servers
const int port = 6969;                           // Port des Servers Senden

// Schräubchen zum drehen
#define FRAMES_PER_PACKET 4  // Anzahl der bytes in einem Packet (jedes byte 8 abtastungen)
#define SAMPLING_RATE_MS 15  // eine Abtastung alle x ms

#define QUEUE_SIZE 64
#define BUFFER_SIZE (1000 / (SAMPLING_RATE_MS * FRAMES_PER_PACKET * 8))  // so viele frames, dass man x milliseks puffer hat
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

ConnectionState state = WIFI_CONNECT;

IPAddress server_ip;
unsigned long last_rx = 0;
unsigned long last_ping = 0;

struct __attribute__((packed)) Packet {
  uint8_t status;  // 0 -> normal package, 1 -> server test (echo back), 2 -> ping
  uint8_t signal[FRAMES_PER_PACKET];
};

//bool BUTTON_PRESSED = false;  // muss öfter gecheckt werdem, darum lieber im sample send task
volatile bool NO_SOUND_MODE = false;
volatile bool NO_PRINTER_MODE = false;
volatile bool SELF_CHECK_MODE = false;
volatile bool SERVER_CHECK_MODE = false;
volatile bool RICK_ROLL_MODE = false;

QueueHandle_t sendQueue;      // Kommunikationsschnittstelle von InputTask -> tcpTask
QueueHandle_t playbackQueue;  // Kommunikationsschnittstelle von sortingTask -> outputTask
QueueHandle_t printQueue;     // Kommunikationsschnittstelle von sortingTask -> printTask

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

  sendQueue = xQueueCreate(QUEUE_SIZE, FRAMES_PER_PACKET * sizeof(uint8_t));
  playbackQueue = xQueueCreate(QUEUE_SIZE, FRAMES_PER_PACKET * sizeof(uint8_t));
  printQueue = xQueueCreate(QUEUE_SIZE, FRAMES_PER_PACKET * sizeof(uint8_t));

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
    Serial.printf("queues (size:%d): send %d play %d print %d \n", QUEUE_SIZE, uxQueueMessagesWaiting(sendQueue), uxQueueMessagesWaiting(playbackQueue), uxQueueMessagesWaiting(printQueue));
    //Serial.printf("Heap free: %u Min heap: %u \n", ESP.getFreeHeap(), ESP.getMinFreeHeap());
    //char stats[512];
    //vTaskGetRunTimeStats(stats);
    //Serial.printf("stats: %s", stats);
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
        showSearchingWiFi();
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
        showResolvingDNS();
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
          showConnectTCP();
          client.stop();
          if (client.connect(server_ip, port)) {
            client.setNoDelay(true);
            //client.setTimeout(5);  // z.B. 5ms
            last_rx = millis();
            last_ping = millis();
            state = RUNNING;
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

        if (millis() - last_rx > 5000) {
          hearingNothing();
          Serial.printf("hearing nothing for %ds\n", (millis() - last_rx) / 1000);
          //last_rx = 0;
        }

        /*if (millis() - last_ping > 10000) {
          Serial.println("TCP timeout");
          client.stop();
          state = TCP_CONNECT;
          break;
        }*/

        handlePackets();
        break;
    }
    vTaskDelay(3 / portTICK_PERIOD_MS);
  }
}

void InputTask(void *pvParameters) {
  uint8_t signal[FRAMES_PER_PACKET];

  while (true) {
    // nichts tun solange keine wifi oder server verbindung
    while (state != RUNNING && SELF_CHECK_MODE == false)
      vTaskDelay(500);

    for (int i = 0; i < FRAMES_PER_PACKET; i++) {
      // taste einen frame ab
      signal[i] = 0;
      for (int j = 0; j < 8; j++) {
        signal[i] <<= 1;
        if (digitalRead(BUTTON) == false)  // button pressed
          signal[i]++;
        vTaskDelay(SAMPLING_RATE_MS / portTICK_PERIOD_MS);
      }
    }



    // sende den Frame
    if (SELF_CHECK_MODE) {
      if (xQueueSend(printQueue, signal, 0) != pdPASS)
        Serial.printf("printQueue overflow!!!\n");
      if (xQueueSend(playbackQueue, signal, 0) != pdPASS)
        Serial.printf("playbackQueue overflow!!!\n");

    } else {
      if (xQueueSend(sendQueue, signal, 0) != pdPASS)
        Serial.printf("sendQueue overflow!!!\n");
    }
  }
}

void PlaybackTask(void *pvParameters) {
  uint8_t signal[FRAMES_PER_PACKET];
  bool buffering = true;
  bool sound_on = false;

  while (true) {

    // buffering status ändern oder warten
    if (buffering) {
      if (uxQueueMessagesWaiting(playbackQueue) > BUFFER_SIZE)
        buffering = false;
      else
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }

    // wenn samples abgespielt gespielt werden
    if (buffering == false) {
      if (xQueueReceive(playbackQueue, signal, 0) == pdPASS) {
        playback(signal, &sound_on);
      } else {
        noTone(SPEAKER);
        digitalWrite(LED, LOW);
        buffering = true;
        sound_on = false;
        Serial.printf("RAN OUT OF BUFFER!!!\n");
      }
    }
  }
}

void PrintTask(void *pvParameters) {
  uint8_t signal[FRAMES_PER_PACKET];
  static bool top_line[384];
  static bool bottom_line[384];
  int index = 0;
  bool writing_top_line = true;
  bool something_in_it = false;

  while (true) {
    if (xQueueReceive(printQueue, signal, portMAX_DELAY) == pdPASS) {  // wartet bis neues Element kommt. blockiert die cpu nicht

      for (int i = 0; i < FRAMES_PER_PACKET; i++) {
        uint8_t mask = 0b10000000;

        for (int j = 0; j < 8; j++) {

          int width_in_pixels = 1;  // SAMPLING_RATE_MS / 20 einfügen für alle 20 millisec bedeuten ein pixel druck. ohne rest

          // nichts anfangen, nichts zu drucken
          if ((signal[i] & mask) == false && something_in_it == false)
            width_in_pixels = 0;
          else
            something_in_it = true;

          // erstellt und sammelt die daten für den druck
          while (width_in_pixels > 0) {
            //Serial.printf("%d", event.state);
            if (writing_top_line)
              top_line[index] = (signal[i] & mask);
            else
              bottom_line[index] = (signal[i] & mask);

            index++;
            width_in_pixels--;
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
      }
    }
  }
}