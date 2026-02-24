/*
 * es könnten noch sachen in drucker und fenster stecken bleiben, wenn stream aufhört
 * rickroll einabauen
 * wenn self/server check mode, muss fremde packete ignorieren, sonst buffer overflow
 * FEHLER: Verbinde mit WGlanE (65650) wifi:sta is connecting, cannot set config
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <HardwareSerial.h>
#include <esp_wifi.h>

// WLAN-Zugangsdaten
const char *ssid = "WGlan";
const char *password = "51565735623896715310";
const char *ssid2 = "Leibniz' Hotspot";
const char *password2 = "";


// UDP-Konfiguration
const char *udp_address = "morse.hopto.org";  // IP des Servers
const int udp_port = 6969;                    // Port zum Senden und Empfangen

// Schräubchen zum drehen
#define WINDOW_SIZE 16
#define LOSS_THRESHOLD_PKS 8
#define LOSS_THRESHOLD_MS 500
#define TARGET_RING_BUFFER_MS 1000
#define RING_BUFFER_SIZE 32
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

struct __attribute__((packed)) MorseEvent {
  uint8_t seq;
  uint16_t duration_ms;
  bool state;  // 1 = Ton, 0 = Pause
};

struct __attribute__((packed)) UdpPacket {
  uint8_t session;
  MorseEvent current_event;
  MorseEvent recent_event;
};

struct WindowSlot {
  bool valid;
  MorseEvent event;
};

//bool BUTTON_PRESSED = false;  // muss öfter gecheckt werdem, darum lieber im sample send task
volatile bool NO_SOUND_MODE = false;
volatile bool NO_PRINTER_MODE = false;
volatile bool SELF_CHECK_MODE = false;
volatile bool SERVER_CHECK_MODE = false;
volatile bool RICK_ROLL_MODE = false;

QueueHandle_t udpQueue;       // Kommunikationsschnittstelle von InputTask -> udpTask
QueueHandle_t sortQueue;      // Kommunikationschnittstelle von udpTask -> sortTask
QueueHandle_t playbackQueue;  // Kommunikationsschnittstelle von sortingTask -> outputTask
QueueHandle_t printQueue;     // Kommunikationsschnittstelle von sortingTask -> printTask

WiFiUDP udp;
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

  udpQueue = xQueueCreate(32, sizeof(UdpPacket));
  sortQueue = xQueueCreate(32, sizeof(UdpPacket));
  playbackQueue = xQueueCreate(32, sizeof(MorseEvent));
  printQueue = xQueueCreate(32, sizeof(MorseEvent));

  xTaskCreate(CheckerTask, "Checkt WiFi und Pin modes", 4068, NULL, 1, NULL);
  xTaskCreate(InputTask, "Input Task", 4096, NULL, 1, NULL);
  xTaskCreate(UdpTask, "udp Task", 4096, NULL, 1, NULL);
  xTaskCreate(SortingTask, "Sorting Task", 4096, NULL, 1, NULL);
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
    testMosfet(); // contains 100ms pause
    checkPins();
    checkWiFi();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void InputTask(void *pvParameters) {
  MorseEvent current_event;
  MorseEvent recent_event;
  UdpPacket packet;
  bool current_state = false;
  bool recent_state = false;
  uint8_t session = esp_random() & 0xFF;
  uint8_t seq = 4;
  uint16_t duration_ms;
  unsigned long recent_ms = millis();

  while (true) {
    current_state = !digitalRead(BUTTON);
    duration_ms = millis() - recent_ms;
    if (current_state != recent_state || duration_ms >= 200) {
      recent_event = current_event;  // memcpy
      current_event.duration_ms = duration_ms;
      current_event.state = recent_state;  // für das gerade beendete event
      current_event.seq = seq;
      packet.session = session;
      packet.current_event = current_event;
      packet.recent_event = recent_event;
      if (xQueueSend(udpQueue, &packet, 0) != pdPASS) {
        Serial.printf("udpSendQueue pass!!!\n");
      }
      recent_state = current_state;
      recent_ms = millis();
      seq++;
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);  // andere lösung? jetzt ist duration_ms nugenutzter speicher, weil nur in 5er schritten auftritt, aber schnellstes tippen ist 50ms lang
  }
}

void UdpTask(void *pvParameters) {
  UdpPacket outgoing;
  UdpPacket incoming;

  while (true) {
    if (WiFi.status() == WL_CONNECTED) {
      // Packete empfangen
      int packet_size = udp.parsePacket();
      if (packet_size >= sizeof(UdpPacket)) {
        udp.read((uint8_t *)&incoming, sizeof(UdpPacket));
        Serial.printf("received sess:%d seq:%d dur:%d stat:%d \n", incoming.session, incoming.current_event.seq, incoming.current_event.duration_ms, incoming.current_event.state);
        if (xQueueSend(sortQueue, &incoming, 0) != pdPASS) {
          Serial.printf("sortQueue pass!!!\n");
        }
      }
      // Packete senden
      if (xQueueReceive(udpQueue, &outgoing, 0) == pdPASS) {
        if (SERVER_CHECK_MODE || SELF_CHECK_MODE) {
          outgoing.session = 0;
        }
        if (SELF_CHECK_MODE)
          udp.beginPacket("127.0.0.1", udp_port);  // 127.0.0.1 ist man selber im wlan modul
        else
          udp.beginPacket(udp_address, udp_port);
        udp.write((uint8_t *)&outgoing, sizeof(UdpPacket));
        udp.endPacket();
      }
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);   // klein, weil chat sagt, udp buffer relativ klein (bei burst doof)
  }
}

void SortingTask(void *pvParameters) {
  UdpPacket packet;
  // stuff for sorting
  WindowSlot sliding_window[WINDOW_SIZE];
  memset(sliding_window, 0, sizeof(sliding_window));
  uint8_t expected_seq = 0;
  uint8_t expected_session = esp_random() & 0xFF;

  // stuff for managing loss
  uint8_t highest_seq_seen = 0;
  unsigned long last_time_sent = millis();

  while (true) {
    // einsortieren
    if (xQueueReceive(sortQueue, &packet, 0) == pdPASS) {
      // im self- oder server check mode sollen fremde packete ignoriert werden
      if ((SELF_CHECK_MODE || SERVER_CHECK_MODE) && packet.session != 0)
        continue;
      
      // session wechsel
      if (packet.session != expected_session) {
        Serial.printf("Session changed!\n");
        expected_session = packet.session;
        expected_seq = packet.current_event.seq;
        highest_seq_seen = expected_seq;
      }
      // recent package
      int8_t pos = (int8_t)(packet.recent_event.seq - expected_seq);  // neg = zu alt, pos = zu früh, 0 = expected
      if (pos >= WINDOW_SIZE) {
        Serial.printf("way to early package: +%d \n", pos);
      } else if (pos >= 0) {
        sliding_window[pos].event = packet.recent_event;
        sliding_window[pos].valid = true;
      }
      // current package
      pos = packet.current_event.seq - expected_seq;  // neg = zu alt, pos = zu früh, 0 = expected
      if ((int8_t)(packet.current_event.seq - highest_seq_seen) > 0) {
        highest_seq_seen = packet.current_event.seq;
      }
      if (pos >= WINDOW_SIZE) {
        Serial.printf("way to early package: +%d \n", pos);
      } else if (pos >= 0) {
        sliding_window[pos].event = packet.current_event;
        sliding_window[pos].valid = true;
      }
    }

    // übergeben ggf loss managen
    if (sliding_window[0].valid) {
      send_and_shift_window(&sliding_window[0]);
      expected_seq++;
      last_time_sent = millis();
    } else if ((int8_t)(highest_seq_seen - expected_seq) >= LOSS_THRESHOLD_PKS) {
      Serial.printf("highest: %d \n", (highest_seq_seen - expected_seq));
      sliding_window[0].event.duration_ms = 10;
      sliding_window[0].event.state = false;
      sliding_window[0].event.seq = expected_seq;
      send_and_shift_window(&sliding_window[0]);
      expected_seq++;
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void PlaybackTask(void *pvParameters) {
  MorseEvent ring_buffer[RING_BUFFER_SIZE];
  int write_index = 0;
  int read_index = 0;
  int buffered_packets = 0;
  int buffered_ms = 0;
  bool buffering = true;
  bool sound_on = false;
  while (true) {
    while (buffered_packets < RING_BUFFER_SIZE && xQueueReceive(playbackQueue, &ring_buffer[write_index], 0) == pdPASS) {
      buffered_ms += ring_buffer[write_index].duration_ms;
      buffered_packets++;
      write_index = (write_index + 1) % RING_BUFFER_SIZE;
    }
    if (buffering) {
      if (buffered_ms > TARGET_RING_BUFFER_MS || buffered_packets == RING_BUFFER_SIZE) {
        buffering = false;
      } else {
        vTaskDelay(5 / portTICK_PERIOD_MS);
      }
    }
    if (buffering == false) {
      if (buffered_packets > 0) {
        playback(&ring_buffer[read_index], &sound_on);
        buffered_ms -= ring_buffer[read_index].duration_ms;
        buffered_packets--;
        read_index = (read_index + 1) % RING_BUFFER_SIZE;
      } else {
        noTone(SPEAKER);
        buffering = true;
      }
    }
  }
}

void PrintTask(void *pvParameters) {
  MorseEvent event;
  static bool top_line[384];
  static bool bottom_line[384];
  int index = 0;
  bool writing_top_line = true;
  bool something_in_it = false;

  while (true) {
    if (xQueueReceive(printQueue, &event, portMAX_DELAY) == pdPASS) {  // wartet bis neues Element kommt. blockiert die cpu nicht
      int width_in_pixels = event.duration_ms / 10;                    // alle 10 millisec bedeuten ein pixel druck. ohne rest

      // nichts anfangen, nichts zu drucken
      if (event.state == false && something_in_it == false) {
        width_in_pixels = 0;
      } else {
        something_in_it = true;
      }

      // erstellt und sammelt die daten für den druck
      while (width_in_pixels > 0) {
        //Serial.printf("%d", event.state);
        if (writing_top_line) {
          top_line[index] = event.state;
        } else {
          bottom_line[index] = event.state;
        }
        index++;
        width_in_pixels--;
        if (index == 384) {
          if (writing_top_line == true) {
            writing_top_line = false;
            index = 0;
          } else {
            if (NO_PRINTER_MODE == false) {
              print(top_line, bottom_line);
            }
            something_in_it = false;
            writing_top_line = true;
            index = 0;
          }
        }
      }
    }
  }
}