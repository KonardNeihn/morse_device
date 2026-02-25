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
#define NUM_BLOCKS 2         // für interleaving
#define FRAMES_PER_BLOCK 4   // 4 Packete ergeben einen FEC Block, für jeden Block ein XOR
#define SAMPLES_PER_FRAME 4  // Anzahl der Abtastungen in einem Packet
#define SAMPLING_RATE_MS 5   // eine Abtastung

#define BUFFER_SIZE (FRAMES_PER_BLOCK * NUM_BLOCKS)
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

struct __attribute__((packed)) Packet {
  uint8_t type;  // 0 = DATA, 1 = FEC
  uint8_t block_id;
  uint8_t frame_id;  // 0-3 bei DATA, 4 bei FEC
  uint32_t timestamp;
  uint8_t signal;
};

struct Block {
  uint8_t block_id;
  bool received[FRAMES_PER_BLOCK + 1];
  Packet packets[FRAMES_PER_BLOCK + 1];
  uint32_t first_seen_time;
  bool ready;
};

struct LossStats {
  uint32_t total_frames;
  uint32_t lost_frames;
  uint32_t recovered_frames;
  uint32_t burst_losses;
};

LossStats stats = { 0 };

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

  udpQueue = xQueueCreate(BUFFER_SIZE, sizeof(Packet));
  sortQueue = xQueueCreate(BUFFER_SIZE, sizeof(Packet));
  playbackQueue = xQueueCreate(BUFFER_SIZE, sizeof(uint8_t));
  printQueue = xQueueCreate(BUFFER_SIZE, sizeof(uint8_t));

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
    testMosfet();  // contains 100ms pause
    checkPins();
    checkWiFi();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void InputTask(void *pvParameters) {
  Packet blocks[NUM_BLOCKS][FRAMES_PER_BLOCK];

  uint8_t write_index[NUM_BLOCKS];
  uint8_t block_id[NUM_BLOCKS];

  // pre shift für round robin
  for (int i = 0; i < NUM_BLOCKS; i++) {
    write_index[i] = (i * FRAMES_PER_BLOCK) / NUM_BLOCKS;
    block_id[i] = i;
  }

  uint8_t current_block = 0;

  while (true) {
    // Frame ins Ringbuffer schreiben
    blocks[current_block][write_index[current_block]].type = 0;
    blocks[current_block][write_index[current_block]].block_id = block_id[current_block];
    blocks[current_block][write_index[current_block]].frame_id = write_index[current_block];
    blocks[current_block][write_index[current_block]].timestamp = millis();
    blocks[current_block][write_index[current_block]].signal = sampleSignal();

    // sende den Frame
    if (xQueueSend(udpQueue, &blocks[current_block][write_index[current_block]], 0) != pdPASS) {
      Serial.printf("udpSendQueue pass!!!\n");
    }

    write_index[current_block]++;

    // Block fertig → FEC berechnen
    if (write_index[current_block] == FRAMES_PER_BLOCK) {
      Packet fec;
      fec.type = 1;
      fec.block_id = block_id[current_block];
      fec.frame_id = FRAMES_PER_BLOCK;
      fec.timestamp = millis();
      uint8_t xor_signal = 0;
      for (int i = 0; i < FRAMES_PER_BLOCK; i++) {
        xor_signal ^= blocks[current_block][i].signal;
      }
      fec.signal = xor_signal;

      // sende den Frame
      if (xQueueSend(udpQueue, &fec, 0) != pdPASS) {
        Serial.printf("udpSendQueue pass!!!\n");
      }

      // Reset Block
      write_index[current_block] = 0;
      block_id[current_block]++;
    }

    current_block = (current_block + 1) % NUM_BLOCKS;  // Nächstes Block für Interleaving
  }
}

// samples aufnehmen
uint8_t sampleSignal() {
  uint8_t signal = 0;
  for (int i = 0; i < SAMPLES_PER_FRAME; i++) {
    signal <<= 1;
    if (digitalRead(BUTTON) == false)  // button pressed
      signal++;
    vTaskDelay(SAMPLING_RATE_MS / portTICK_PERIOD_MS);
  }
  return signal;
}

void UdpTask(void *pvParameters) {
  Packet outgoing;
  Packet incoming;

  while (true) {
    if (WiFi.status() == WL_CONNECTED) {
      // Packete empfangen
      int packet_size = udp.parsePacket();
      if (packet_size >= sizeof(Packet)) {
        udp.read((uint8_t *)&incoming, sizeof(Packet));
        Serial.printf("received type:%d block:%d frame:%d time:%d signal:%d \n", incoming.type, incoming.block_id, incoming.frame_id, incoming.timestamp, incoming.signal);
        if (xQueueSend(sortQueue, &incoming, 0) != pdPASS) {
          Serial.printf("sortQueue pass!!!\n");
        }
      }
      // Packete senden
      if (xQueueReceive(udpQueue, &outgoing, 0) == pdPASS) {
        if (SELF_CHECK_MODE)
          udp.beginPacket("127.0.0.1", udp_port);  // 127.0.0.1 ist man selber im wlan modul
        else
          udp.beginPacket(udp_address, udp_port);
        udp.write((uint8_t *)&outgoing, sizeof(Packet));
        udp.endPacket();
      }
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);  // klein, weil chat sagt, udp buffer relativ klein (bei burst doof)
  }
}

void SortingTask(void *pvParameters) {

  Block blocks[NUM_BLOCKS];
  uint32_t next_play_seq = 0;
  Packet packet;

  // einmal alle block ids ungültig machen
  for (int i = 0; i < NUM_BLOCKS; i++) {
    blocks[i].block_id = 255;  // ungültig
  }

  while (true) {
    if (xQueueReceive(sortQueue, &packet, portMAX_DELAY) == pdPASS) {
      uint8_t block_id = packet.block_id % NUM_BLOCKS;

      // block reset, falls neuer block
      if (packet.block_id != blocks[block_id].block_id) {
        if (blocks[block_id].ready == false) {
          fillLosses(&blocks[block_id]);
          tryPlayback(blocks, &next_play_seq);
        }
        resetBlock(&blocks[block_id], packet.block_id);  // hier wäre ein guter moment, um einen loss zu akzeptieren bzw muss ja
      }

      blocks[block_id].packets[packet.frame_id] = packet;
      blocks[block_id].received[packet.frame_id] = true;

      checkBlockReady(&blocks[block_id]);
      tryPlayback(blocks, &next_play_seq);
    }
  }
}

void fillLosses(Block *block) {
  for (int i = 0; i < FRAMES_PER_BLOCK; i++) {
    if (block->received[i] == false) {
      block->packets[i].signal = 0b01010101;
      block->ready = true;
      stats.lost_frames++;
    }
  }
}

void resetBlock(Block *block, uint8_t new_id) {
  block->block_id = new_id;
  block->ready = false;
  block->first_seen_time = millis();

  for (int i = 0; i <= FRAMES_PER_BLOCK; i++)
    block->received[i] = false;
}

void checkBlockReady(Block *block) {
  int missing = -1;
  int missing_count = 0;

  for (int i = 0; i < FRAMES_PER_BLOCK; i++) {
    if (block->received[i] == false) {
      missing = i;
      missing_count++;
    }
  }

  // wenn alle frames da und kein FEC gebraucht wird
  if (missing_count == 0) {
    block->ready = true;
    return;
  }

  // fec schon da?
  bool fec_present = block->received[FRAMES_PER_BLOCK];

  // wenn rekonstruiert werden kann
  if (missing_count == 1 && fec_present) {
    uint8_t xor_signal = block->packets[FRAMES_PER_BLOCK].signal;

    for (int i = 0; i < FRAMES_PER_BLOCK; i++) {
      if (i != missing)
        xor_signal ^= block->packets[i].signal;
    }

    block->packets[missing].signal = xor_signal;
    block->received[missing] = true;
    block->ready = true;
    stats.recovered_frames++;
    return;
  }
  // Mehr als 1 fehlt → nicht ready
}

void tryPlayback(Block blocks[], uint32_t *next_play_seq) {
  while (true) {
    uint8_t expected_block = *next_play_seq / FRAMES_PER_BLOCK;
    uint8_t expected_frame = *next_play_seq % FRAMES_PER_BLOCK;
    uint8_t block_id = expected_block % NUM_BLOCKS;

    if (blocks[block_id].block_id != expected_block)
      return;

    if (blocks[block_id].ready && blocks[block_id].received[expected_frame]) {

      if (xQueueSend(playbackQueue, &blocks[block_id].packets[expected_frame].signal, 0) != pdPASS) {
        Serial.printf("playbackQueue pass!!!\n");
      }
      if (xQueueSend(printQueue, &blocks[block_id].packets[expected_frame].signal, 0) != pdPASS) {
        Serial.printf("printQueue pass!!!\n");
      }
      (*next_play_seq)++;
      continue;
    }
    return;  // warten
  }
}

void PlaybackTask(void *pvParameters) {
  uint8_t ring_buffer[BUFFER_SIZE];
  int write_index = 0;
  int read_index = 0;
  int buffered_packets = 0;
  bool buffering = true;
  bool sound_on = false;

  while (true) {

    // so viele packete lesen, wies geht
    while (buffered_packets < BUFFER_SIZE && xQueueReceive(playbackQueue, &ring_buffer[write_index], 0) == pdPASS) {
      buffered_packets++;
      write_index = (write_index + 1) % BUFFER_SIZE;
    }

    // buffering status ändern oder warten
    if (buffering) {
      if (buffered_packets > BUFFER_SIZE / 2) {
        buffering = false;
      } else {
        vTaskDelay(5 / portTICK_PERIOD_MS);
      }
    }

    // wenn samples abgespielt gespielt werden
    if (buffering == false) {
      if (buffered_packets > 0) {
        playback(&ring_buffer[read_index], &sound_on);
        buffered_packets--;
        read_index = (read_index + 1) % BUFFER_SIZE;
      } else {
        noTone(SPEAKER);
        buffering = true;
      }
    }
  }
}

void PrintTask(void *pvParameters) {
  uint8_t signal;
  static bool top_line[384];
  static bool bottom_line[384];
  int index = 0;
  bool writing_top_line = true;
  bool something_in_it = false;


  while (true) {
    if (xQueueReceive(printQueue, &signal, portMAX_DELAY) == pdPASS) {  // wartet bis neues Element kommt. blockiert die cpu nicht
      uint8_t mask = 0b00000001;
      for (int i = 0; i < SAMPLES_PER_FRAME; i++) {

        int width_in_pixels = SAMPLING_RATE_MS / 10;  // alle 10 millisec bedeuten ein pixel druck. ohne rest

        // nichts anfangen, nichts zu drucken
        if ((signal & mask) && something_in_it == false) {
          width_in_pixels = 0;
        } else {
          something_in_it = true;
        }

        // erstellt und sammelt die daten für den druck
        while (width_in_pixels > 0) {
          //Serial.printf("%d", event.state);
          if (writing_top_line) {
            top_line[index] = (signal & mask);
          } else {
            bottom_line[index] = (signal & mask);
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
        mask <<= 1;
      }
    }
  }
}