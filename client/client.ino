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
const char *ssid = "WGlan";
const char *password = "51565735623896715310";
const char *ssid2 = "Leibniz' Hotspot";
const char *password2 = "";


// Server-Konfiguration
const char *server_address = "morse.hopto.org";  // IP des Servers
const int port = 6969;                           // Port zum Senden und Empfangen

// Schräubchen zum drehen
#define SAMPLES_PER_FRAME 10  // Anzahl der Abtastungen in einem Packet (max 32)
#define SAMPLING_RATE_MS 5    // eine Abtastung

#define BUFFER_SIZE (1000 / (SAMPLING_RATE_MS * SAMPLES_PER_FRAME))  // so viele frames, dass man eine sekunde puffer hat
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

struct __attribute__((packed)) Packet {
  uint8_t status;
  uint32_t signal;
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
  pinMode(BUTTON, INPUT);
  pinMode(MOSFET, OUTPUT);
  pinMode(NORMAL_MODE_PIN, INPUT);
  pinMode(NO_SOUND_MODE_PIN, INPUT);
  pinMode(NO_PRINTER_MODE_PIN, INPUT);
  pinMode(SELF_CHECK_MODE_PIN, INPUT);
  pinMode(SERVER_CHECK_MODE_PIN, INPUT);
  pinMode(RICK_ROLL_MODE_PIN, INPUT);

  Serial.begin(115200);
  printer.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  sendQueue = xQueueCreate(BUFFER_SIZE, sizeof(uint32_t));
  playbackQueue = xQueueCreate(BUFFER_SIZE, sizeof(uint32_t));
  printQueue = xQueueCreate(BUFFER_SIZE, sizeof(uint32_t));

  xTaskCreate(CheckerTask, "Checkt WiFi und Pin modes", 4068, NULL, 1, NULL);
  xTaskCreate(InputTask, "Input Task", 4096, NULL, 1, NULL);
  xTaskCreate(TcpTask, "tcp Task", 4096, NULL, 1, NULL);
  xTaskCreate(PlaybackTask, "Output Task", 4096, NULL, 1, NULL);
  xTaskCreate(PrintTask, "Print Task", 4096, NULL, 1, NULL);

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
    Serial.printf("queues: send %d play %d print %d\n", uxQueueMessagesWaiting(sendQueue), uxQueueMessagesWaiting(playbackQueue), uxQueueMessagesWaiting(printQueue));
  }
}

void ConnectionTask(void *pvParameters) {
  int wichWiFi = 0;
  while (true) {
    switch (state) {

      case WIFI_CONNECT:
        Serial.println("Connecting WiFi");

        Serial.printf("Connecting to ");
        if (wichWiFi == 0)
          Serial.println(ssid);
        else
          Serial.println(ssid2);

        WiFi.disconnect(true, true);
        WiFi.mode(WIFI_OFF);
        vTaskDelay(500 / portTICK_PERIOD_MS);

        WiFi.mode(WIFI_STA);
        if (wichWiFi == 0)
          WiFi.begin(ssid, password);
        else
          WiFi.begin(ssid2, password2);

        if (WiFi.waitForConnectResult() == WL_CONNECTED) {
          WiFi.setSleep(false);
          esp_wifi_set_ps(WIFI_PS_NONE);
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
        Serial.println("Connecting TCP");
        client.stop();
        if (client.connect(server_ip, port)) {
          client.setNoDelay(true);
          last_rx = millis();
          state = RUNNING;
          Serial.println("TCP connected");
        } else {
          Serial.println("TCP failed");
          vTaskDelay(2000 / portTICK_PERIOD_MS);
        }
        break;

      case RUNNING:
        if (WiFi.status() != WL_CONNECTED) {
          Serial.println("WiFi lost");
          state = WIFI_CONNECT;
          break;
        }

        if (client.connected() == false) {
          Serial.println("TCP lost");
          state = TCP_CONNECT;
          break;
        }

        if (millis() - last_rx > 15000) {
          Serial.println("TCP timeout");
          client.stop();
          state = TCP_CONNECT;
          break;
        }
        break;
    }
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

void InputTask(void *pvParameters) {
  uint32_t signal = 0;

  while (true) {
    // nichts tun solange keine wifi oder server verbindung
    while (state != RUNNING)
      vTaskDelay(500);

    // taste einen frame ab
    for (int i = 0; i < SAMPLES_PER_FRAME; i++) {
      signal <<= 1;
      if (digitalRead(BUTTON) == false)  // button pressed
        signal++;
      vTaskDelay(SAMPLING_RATE_MS / portTICK_PERIOD_MS);
    }

    // sende den Frame
    if (SELF_CHECK_MODE) {
      if (xQueueSend(printQueue, &signal, 0) != pdPASS)
        Serial.printf("printQueue pass!!!\n");
      if (xQueueSend(playbackQueue, &signal, 0) != pdPASS)
        Serial.printf("playbackQueue pass!!!\n");

    } else {
      if (xQueueSend(sendQueue, &signal, 0) != pdPASS)
        Serial.printf("sendQueue pass!!!\n");
    }
  }
}

void TcpTask(void *pvParameters) {
  Packet outgoing;
  Packet incoming;

  while (true) {
    // nichts tun solange keine wifi oder server verbindung
    while (state != RUNNING)
      vTaskDelay(500);

    if (SERVER_CHECK_MODE)
      outgoing.status = 1;
    else
      outgoing.status = 0;

    // Packete empfangen
    while (client.available() >= sizeof(Packet)) {
      client.read((uint8_t *)&incoming, sizeof(Packet));
      last_rx = millis();

      if (xQueueSend(printQueue, &incoming.signal, 0) != pdPASS)
        Serial.printf("printQueue pass!!!\n");

      if (xQueueSend(playbackQueue, &incoming.signal, 0) != pdPASS)
        Serial.printf("playbackQueue pass!!!\n");
    }

    // Packete senden
    if (xQueueReceive(sendQueue, &outgoing.signal, 0) == pdPASS)
      client.write((uint8_t *)&outgoing, sizeof(Packet));

    vTaskDelay(5 / portTICK_PERIOD_MS);  // klein, weil chat sagt, udp buffer relativ klein (bei burst doof)
  }
}

void PlaybackTask(void *pvParameters) {
  uint32_t frame;
  bool buffering = true;
  bool sound_on = false;

  while (true) {

    // buffering status ändern oder warten
    if (buffering) {
      if (uxQueueMessagesWaiting(playbackQueue) > BUFFER_SIZE / 2)
        buffering = false;
      else
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }

    // wenn samples abgespielt gespielt werden
    if (buffering == false) {
      if (uxQueueMessagesWaiting(playbackQueue) > 0) {
        xQueueReceive(sendQueue, &frame, 0);
        playback(&frame, &sound_on);
      } else {
        noTone(SPEAKER);
        buffering = true;
      }
    }
  }
}

void PrintTask(void *pvParameters) {
  uint32_t signal;
  static bool top_line[384];
  static bool bottom_line[384];
  int index = 0;
  bool writing_top_line = true;
  bool something_in_it = false;

  while (true) {
    if (xQueueReceive(printQueue, &signal, portMAX_DELAY) == pdPASS) {  // wartet bis neues Element kommt. blockiert die cpu nicht
      uint32_t mask = 1;
      for (int i = 0; i < SAMPLES_PER_FRAME; i++) {

        int width_in_pixels = SAMPLING_RATE_MS / 10;  // alle 10 millisec bedeuten ein pixel druck. ohne rest

        // nichts anfangen, nichts zu drucken
        if ((signal & mask) && something_in_it == false)
          width_in_pixels = 0;
        else
          something_in_it = true;

        // erstellt und sammelt die daten für den druck
        while (width_in_pixels > 0) {
          //Serial.printf("%d", event.state);
          if (writing_top_line)
            top_line[index] = (signal & mask);
          else
            bottom_line[index] = (signal & mask);

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
        mask <<= 1;
      }
    }
  }
}