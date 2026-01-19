/*
 * COULD DO
 * programm für computer schreiben
 * vllt doch tcp?
 * lückenerkennung - wahrscheinlich zeug einfügen
 * debug länger drücken
 * 
 * DRUCK
 * Schraubenlöcher?
 * Alphabet wohin?
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <HardwareSerial.h>

#define BUZZER 12
#define BUTTON 14
#define LED 13
#define TX_PIN 17
#define RX_PIN 16
#define NORMAL_MODE_PIN 27
#define NO_BUZZER_MODE_PIN 26
#define NO_PRINTER_MODE_PIN 25
#define SELF_CHECK_MODE_PIN 33
#define SERVER_CHECK_MODE_PIN 32
#define OPT_MODE_PIN 35

//bool BUTTON_PRESSED = false;  // muss öfter gecheckt werdem, darum lieber im sample send task
volatile bool NO_BUZZER_MODE = false;
volatile bool NO_PRINTER_MODE = false;
volatile bool SELF_CHECK_MODE = false;
volatile bool SERVER_CHECK_MODE = false;

// WLAN-Zugangsdaten
//const char *ssid = "NichtDeins";
//const char *password = "oxygen2025";

const char *ssid = "WGlan";
const char *password = "51565735623896715310";
const char *ssid2 = "Leibniz' Hotspot";
const char *password2 = "";

/*
const char *ssid = "LestMehrBuchen!";
const char *password = "fluessigesHelium-268,8)";
const char *ssid2 = "NichtDeins";
const char *password2 = "oxygen2025";
*/

// UDP-Konfiguration
const char *udpAddress = "morse.hopto.org";  // IP des Servers
const int udpPort = 6969;                    // Port zum Senden und Empfangen

// Buffer-Konfiguration
const int BUFFER_SIZE = 8;
const int PACKET_SIZE = 8;  // Max Größe eines Pakets (PLAYBACK_INTERVAL / SAMPLE_RATE)

struct __attribute__((packed)) UdpPacket {  // __attribute__((packed)) sorgt dafür, dass es genauso im speiche liegt, ohne lücken etc. wichtig bei der packet aufnahme vom wifi modul
  uint8_t session;
  uint8_t seq;
  uint8_t data;
};

QueueHandle_t printQueue;     // eine Kommunikationsschnittstelle zwischen dem receiveTask -> printTask
QueueHandle_t udpSendQueue;   // eine Kommunikationsschnittstelle zwischen dem sampleSendTask -> udptask
QueueHandle_t udpRecvQueue;   // eine Kommunikationsschnittstelle zwischen dem udpTask -> receiveTask
QueueHandle_t playbackQueue;  // eine Kommunikationsschnittstelle zwischen dem receiveTask -> playBackTask

// Intervall zur Ausgabe und Aufnahme
//const int PLAYBACK_INTERVAL = 40; // in ms
const int SAMPLE_RATE = 10;                          // in ms
const int TURN_OFF_MAX = BUFFER_SIZE * PACKET_SIZE;  // turns off transmission after x empty samplings, halbe bufferlänge, damit keine töne im buffer stecken bleiben
const int PING_INTERVAL = 100;                       // sends ping after x not sent packages NOT SAMPLINGS

const int PRINT_TIMEOUT = 5000;  // wenn etwas empfangen wurde, was keine ganze zeile ausfüllt, soll es ausgedruckt werden

const int BUZZER_FREQ = 200;

const int WARM_UP_PACKETS = 3;

WiFiUDP udp;
HardwareSerial printer(1);

UBaseType_t updHighwater;
UBaseType_t pinHighwater;
UBaseType_t receiveHighwater;
UBaseType_t sampleSendHighwater;
UBaseType_t playBackHighwater;
UBaseType_t WiFiMonitorHighwater;
UBaseType_t printHighwater;

void setup() {
  pinMode(BUZZER, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(BUTTON, INPUT);
  pinMode(NORMAL_MODE_PIN, INPUT);
  pinMode(NO_BUZZER_MODE_PIN, INPUT);
  pinMode(NO_PRINTER_MODE_PIN, INPUT);
  pinMode(SELF_CHECK_MODE_PIN, INPUT);
  pinMode(SERVER_CHECK_MODE_PIN, INPUT);
  pinMode(OPT_MODE_PIN, INPUT);

  printer.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial.begin(115200);

  vTaskDelay(1000 / portTICK_PERIOD_MS);  //wartezeit für serielle dings, aber wlan dauert schon lange genug

  checkWiFi();

  printer.write(27);
  printer.write('@');  // ESC @ means reset
  printer.write(27);
  printer.write(55);   // ESC 7 configure heating parameters
  printer.write(1);    // n1 = 5   → heizpunkte, die gleichzeitig laufen dürfen
  printer.write(255);  // n2 = 240 → lange Heizzeit (langsam, aber dunkler)
  printer.write(255);  // n3 = 200 → große Pause zwischen Heizungen, Strom sinkt stark
  // maximaler energiesparmodus, um auch mit 500mA USB auszukommen

  if (digitalRead(NO_PRINTER_MODE_PIN) != LOW) {  // nur wenn der drehschalter nicht auf NO_PRINTING steht
    //printer.println("(debug) esp booted");
  }

  udpSendQueue = xQueueCreate(32, sizeof(UdpPacket));
  udpRecvQueue = xQueueCreate(32, sizeof(UdpPacket));
  playbackQueue = xQueueCreate(32, sizeof(uint8_t));
  printQueue = xQueueCreate(96, sizeof(uint8_t));  // 96 sind zwei zeilen des druckers

  // Tasks erstellen
  xTaskCreatePinnedToCore(WiFiMonitorTask, "WiFi Monitor Task", 4096, NULL, 1, NULL, 1);
  xTaskCreate(pinReadTask, "Pin Read Task", 2048, NULL, 1, NULL);
  xTaskCreatePinnedToCore(udpWorkerTask, "UDP Worker Task", 4096, NULL, 3, NULL, 0);
  xTaskCreate(distributeTask, "Distribute Task", 2048, NULL, 2, NULL);
  xTaskCreate(sampleTask, "Sample Task", 2048, NULL, 2, NULL);
  xTaskCreate(playbackTask, "Play Task", 2048, NULL, 2, NULL);
  xTaskCreatePinnedToCore(printTask, "Print Task", 12288, NULL, 1, NULL, 1);

  //vTaskDelete(NULL);  // Beendet den Arduino-Loop-Task
  for (;;) vTaskDelay(portMAX_DELAY);
}

void loop() {}

void playTone() {
  if (!NO_BUZZER_MODE)          // nur wenn der drehschalter nicht "ohne Ton" sagt (LOW = angeschaltet)
    tone(BUZZER, BUZZER_FREQ);  // tone() blockiert auf dem esp32 den thread NICHT
}

void beepOnce() {
  playTone();
  digitalWrite(LED, HIGH);
  vTaskDelay(150 / portTICK_PERIOD_MS);
  noTone(BUZZER);
  digitalWrite(LED, LOW);
  vTaskDelay(850 / portTICK_PERIOD_MS);
}

void beepTwice() {
  playTone();
  digitalWrite(LED, HIGH);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  noTone(BUZZER);
  digitalWrite(LED, LOW);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  playTone();
  digitalWrite(LED, HIGH);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  noTone(BUZZER);
  digitalWrite(LED, LOW);
}

// loopt solange, bis eins von zwei wlans verbunden ist
void checkWiFi() {
  while (WiFi.status() != WL_CONNECTED) {
    if (digitalRead(NO_BUZZER_MODE_PIN) == LOW)
      NO_BUZZER_MODE = true;
    connectWiFi(ssid, password);
    if (WiFi.status() != WL_CONNECTED)
      connectWiFi(ssid2, password2);
  }
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

// versucht sich mit einem WLAN zu verbinden
void connectWiFi(const char *ssid, const char *password) {
  int connect_attempt = 0;  // verbindungs timeout
  Serial.print("Verbinde mit ");
  Serial.print(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED && connect_attempt <= 5) {
    beepOnce();
    Serial.print(".");
    connect_attempt++;
  }
  Serial.println("");
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Couldn't connect!");
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);  //sonst kann keine neue ssid & passwd vergeben werden
    vTaskDelay(500 / portTICK_PERIOD_MS);
    WiFi.mode(WIFI_STA);
  } else {
    Serial.println("\nWLAN Verbunden!");
    Serial.printf("IP-Adresse: %d.%d.%d.%d Empfang: %ddb\n", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3], WiFi.RSSI());
    beepTwice();
    WiFi.setSleep(false);  //um hoffentlich package loss zu verhindern
    udp.stop();
    udp.begin(udpPort);
    Serial.printf("Lausche und sende vermutlich nicht auf Port: %d haha\n", udpPort);
  }
}

/*
 * dieser Thread setzt global flags, welcher Modus gerade ist
 */
void pinReadTask(void *pvParameters) {
  while (true) {
    NO_BUZZER_MODE = (digitalRead(NO_BUZZER_MODE_PIN) == LOW);
    NO_PRINTER_MODE = (digitalRead(NO_PRINTER_MODE_PIN) == LOW);
    SELF_CHECK_MODE = (digitalRead(SELF_CHECK_MODE_PIN) == LOW);
    SERVER_CHECK_MODE = (digitalRead(SERVER_CHECK_MODE_PIN) == LOW);

    pinHighwater = uxTaskGetStackHighWaterMark(NULL);
    taskYIELD();
  }
}

/*
 * Dieser Thread ist dazu da, um in bestimmten abständen etwas zu machen zb wlan verbindung checken
 */
void WiFiMonitorTask(void *pvParameters) {
  while (true) {
    checkWiFi();
    //char buffer[1024];
    //vTaskGetRunTimeStats(buffer);

    WiFiMonitorHighwater = uxTaskGetStackHighWaterMark(NULL);
    //Serial.printf("Stacks udp: %d pin: %d receive: %d send: %d play: %d wifi: %d print: %d\n", updHighwater, pinHighwater, receiveHighwater, sampleSendHighwater, playBackHighwater, WiFiMonitorHighwater, printHighwater);
    vTaskDelay(5000 / portTICK_PERIOD_MS);  //alle 5 sek das wlan checken
  }
}

/*
 * Dieser Thread arbeitet udp Aufgaben der anderen Tasks seriall ab, da es sonst zu race conditions kommt. übergabe über queues
 */
void udpWorkerTask(void *pvParameters) {
  UdpPacket outgoing;
  UdpPacket incoming;

  while (true) {
    // Empfang prüfen
    int packetSize = udp.parsePacket();
    if (packetSize >= sizeof(UdpPacket)) {
      udp.read((uint8_t *)&incoming, sizeof(UdpPacket));
      //Serial.printf("recieved sess:%u seq:%u data:%u time:%d\n", incoming.session, incoming.seq, incoming.data, millis());

      if (xQueueSend(udpRecvQueue, &incoming, 0) != pdPASS)  // wartet 0 ticks, falls voll
        Serial.println("udp incoming Queue pass!!!");
    }
    // Senden falls gewünscht
    if (xQueueReceive(udpSendQueue, &outgoing, 0) == pdPASS) {  //wartet 0 ticks, falls nichts drin
      if (SERVER_CHECK_MODE || SELF_CHECK_MODE)
        outgoing.session = 0;  // zeichen für den server, dass an selben zurücksenden

      if (SELF_CHECK_MODE)
        udp.beginPacket("127.0.0.1", udpPort);  // 127.0.0.1
      else
        udp.beginPacket(udpAddress, udpPort);
      udp.write((uint8_t *)&outgoing, sizeof(UdpPacket));
      udp.endPacket();

      if (SERVER_CHECK_MODE || SELF_CHECK_MODE)
        outgoing.session = esp_random() & 0xFF;
      //Serial.printf("sent     sess:%u seq:%u data:%u time:%d\n", outgoing.session, outgoing.seq, outgoing.data, millis());
    }
    updHighwater = uxTaskGetStackHighWaterMark(NULL);
    vTaskDelay(2 / portTICK_PERIOD_MS);  // Entspannt CPU-Last und lässt dem netzwerk zeit
  }
}

/*
 * dieser Thread verarbeitet die empfangenen packete für die anderen threads
 */
void distributeTask(void *pvParameters) {
  uint8_t current_session = esp_random() & 0xFF;
  uint8_t expected_seq = 0;
  int8_t seq_diff;

  UdpPacket incoming;

  struct JitterSlot {
    uint8_t data;
    bool valid;
  };

  JitterSlot jitter_window[BUFFER_SIZE];
  memset(jitter_window, 0, sizeof(jitter_window));

  while (true) {
    // gucken, ob neues Packet angekommen ist
    if (xQueueReceive(udpRecvQueue, &incoming, portMAX_DELAY) == pdPASS) {  //wartet für immer, falls nichts drin

      // neue session -> seq_num reset
      if (incoming.session != current_session) {
        Serial.printf("New Session: %d\n", incoming.session);
        // den buffer vollkommen leeren und in die queue schreiben. fehlende packete durch dummy sample ersetzen
        for (int i = 0; i < BUFFER_SIZE - 1; i++) {
          // nicht valide(verlorene) packete ersetzen durch stille
          if (!jitter_window[i].valid)
            jitter_window[i].data = 0;

          // übergeben
          if (xQueueSend(playbackQueue, &jitter_window[i].data, 0) != pdPASS)
            Serial.println("Playback Queue pass!");
          if (xQueueSend(printQueue, &jitter_window[i].data, 0) != pdPASS)
            Serial.println("Print Queue pass!");
          jitter_window[i].valid = false;
        }
        current_session = incoming.session;
        expected_seq = incoming.seq;
      }

      seq_diff = (int8_t)(incoming.seq - expected_seq);  // pos = too new, neg = old, 0 = expected

      // debug
      if (seq_diff != 0)
        Serial.printf("seq_diff: %d\n", seq_diff);

      // zu altes packet (älter) -> verwerfen (rest der schleife überspringen)
      if (seq_diff < 0) continue;

      // so neues packet, dass es mindestens nicht mehr in den buffer passt
      if (seq_diff >= BUFFER_SIZE) {
        Serial.printf("flush\n");
        // den buffer vollkommen leeren und in die queue schreiben. fehlende packete durch dummy sample ersetzen
        for (int i = 0; i < BUFFER_SIZE - 1; i++) {
          // nicht valide(verlorene) packete ersetzen durch stille
          if (!jitter_window[i].valid)
            jitter_window[i].data = 0;

          // übergeben
          if (xQueueSend(playbackQueue, &jitter_window[i].data, 0) != pdPASS)
            Serial.println("Playback Queue pass!");
          if (xQueueSend(printQueue, &jitter_window[i].data, 0) != pdPASS)
            Serial.println("Print Queue pass!");
          jitter_window[i].valid = false;
        }
        seq_diff = 0;
        expected_seq = incoming.seq;
      }

      // packet einsortieren
      jitter_window[seq_diff].data = incoming.data;
      jitter_window[seq_diff].valid = true;

      // alles ausgeben, was bisher lückenlos da ist
      while (jitter_window[0].valid) {
        //übergeben
        if (xQueueSend(playbackQueue, &jitter_window[0].data, 0) != pdPASS)
          Serial.println("Playback Queue pass!");
        if (xQueueSend(printQueue, &jitter_window[0].data, 0) != pdPASS)
          Serial.println("Print Queue pass!");

        //fenster weiterschieben
        for (int i = 0; i < BUFFER_SIZE - 1; i++)
          jitter_window[i] = jitter_window[i + 1];
        jitter_window[BUFFER_SIZE - 1].valid = false;
        expected_seq++;
      }
    }
    receiveHighwater = uxTaskGetStackHighWaterMark(NULL);
    taskYIELD();
  }
}

/*
 * Dieser Thread speichert Samples in einem Packet '1' für an und '0' für aus
 * Wenn das Packet voll ist, wird das packet abgeschickt
 * evtl ping und abstellen, wenn Stille
 * tatsächlich ist der Schalter bei LOW betätigt
 */
void sampleTask(void *pvParameters) {
  UdpPacket packet;
  packet.session = esp_random() & 0xFF;
  packet.seq = 0;
  int zeros = 0;  // counting zeros to turn off while silence
  int pingCounter = 0;
  bool stoppedSending = true;

  while (true) {
    packet.data = 0b00000000;
    for (uint8_t read_mask = 0b00000001; read_mask > 0; read_mask = read_mask << 1) {
      if (digitalRead(BUTTON) == LOW) {
        packet.data = packet.data | read_mask;
        zeros = 0;
        pingCounter = 0;
      } else {
        zeros++;
      }
      vTaskDelay(SAMPLE_RATE / portTICK_PERIOD_MS);  // wartet so viele ticks (zeitscheiben), wie eine sample_rate lang ist
    }
    if (zeros <= TURN_OFF_MAX || pingCounter >= PING_INTERVAL) {
      // wenn wieder neu angefangen wird, zu senden, werden warmup packete vorgesendet
      if (stoppedSending) {
        UdpPacket dummy;
        for (int i = 0; i < WARM_UP_PACKETS; i++) {
          packet.seq++;
          dummy = packet;
          dummy.data = 0;
          if (xQueueSend(udpSendQueue, &dummy, 0) != pdPASS)  // wartet 0 ticks, falls voll
            Serial.println("send Queue pass!!!");
          vTaskDelay(1);
        }
      }
      packet.seq++;
      // übergeben an udp task
      if (xQueueSend(udpSendQueue, &packet, 0) != pdPASS)  // wartet 0 ticks, falls voll
        Serial.println("send Queue pass!!!");
      pingCounter = 0;
      stoppedSending = false;
    } else {
      pingCounter++;
      stoppedSending = true;
    }

    sampleSendHighwater = uxTaskGetStackHighWaterMark(NULL);
    taskYIELD();
  }
}

/*
 * Dieser Thread guckt, ob packete da sind
 * wenn mehr als 5 packete da sind, beginnt er sie abzuspielen und setzt tonePlaying auf TRUE
 * jetzt werden auch packete abgespielt, wenn weniger als 5 da sind
 * wenn die packets leer gehen, wird wieder nichts gespielt, bis wieder 5 packets da sind
 * evtl delay, um cpu zeit freizugeben
 */
void playbackTask(void *pvParameters) {  // sample entpacken
  uint8_t data;
  bool tonePlaying = false;
  while (true) {
    if (xQueueReceive(playbackQueue, &data, portMAX_DELAY) == pdPASS) {
      for (uint8_t read_mask = 0b00000001; read_mask > 0; read_mask = read_mask << 1) {  // die 1 wird solange geschiftet, bis sie weg ist und nur noch 0b00000000 da ist
        if (data & read_mask) {
          if (!tonePlaying) {
            playTone();  // tone() blockiert auf dem esp32 den thread NICHT
            digitalWrite(LED, HIGH);
            tonePlaying = true;
          }
        } else {
          noTone(BUZZER);
          digitalWrite(LED, LOW);
          tonePlaying = false;
        }
        vTaskDelay(SAMPLE_RATE / portTICK_PERIOD_MS);  // wartet so viele ticks (zeitscheiben), wie eine sample_rate lang ist
      }
    }
    playBackHighwater = uxTaskGetStackHighWaterMark(NULL);
    taskYIELD();
  }
}

/*
 * Dieser Thread bekommt die empfangenen samples geschickt und baut mit ihnen eine bzw zwei zeilen
 * wenn genug samples für eine ganze zeile gesammelt wurde, dann wird diese gedruckt
 */

void printTask(void *pvParameters) {
  uint8_t packet;
  static uint8_t line_top[384];
  static uint8_t line_bottom[384];
  memset(line_top, 0, sizeof(line_top));
  memset(line_bottom, 0, sizeof(line_bottom));
  int print_index = 0;  // wo die zeile gerade erstellt wird
  bool top_line = true;
  bool something_in_it = false;  // ob die zeile schon etwas in sich hat
  unsigned long last_added = millis();

  while (true) {
    if (xQueueReceive(printQueue, &packet, portMAX_DELAY) == pdPASS) {                   // wartet bis neues Element kommt. blockiert die cpu nicht
      for (uint8_t read_mask = 0b00000001; read_mask > 0; read_mask = read_mask << 1) {  // die 1 wird solange geschiftet, bis sie weg ist und nur noch 0b00000000 da ist
        // wenn der index die zeile überschreitet und noch keine untere zeile eröffnet wurde
        if (print_index >= 384) {
          if (top_line) {
            print_index = 0;
            top_line = false;
          }
          else
            Serial.println("PRINT INDEX OVERFLOW");
        }
        // wenn eine 1 gelesen wird
        if (packet & read_mask) {
          if (print_index < 384) {
            if (top_line)
              line_top[print_index] = 0b01100000;
            else
              line_bottom[print_index] = 0b00000110;
          }
          else
            Serial.println("PRINT INDEX OVERFLOW");
          something_in_it = true;
          if (print_index < 384)
            print_index++;
          else
            Serial.println("PRINT INDEX OVERFLOW");
          last_added = millis();
        }
        // wenn eine null gelesen wird und schon was geschreiben wurde
        else if (something_in_it) {  // wenn null geschrieben wird, aber schon mindestens eine 1 im buffer ist
          if (print_index < 384) {
            if (top_line)
              line_top[print_index] = 0b00000000;
            else
              line_bottom[print_index] = 0b00000000;
          }
          else
            Serial.println("PRINT INDEX OVERFLOW");
          if (print_index < 384)
            print_index++;
          else
            Serial.println("PRINT INDEX OVERFLOW");
        }

        // abfrage, ob wenn was im speicher ist, aber lange nichts neues mehr gekommen ist, wird index so gesetzt, als ob die zeile voll ist
        if (something_in_it && (millis() - last_added > PRINT_TIMEOUT)) {
          print_index = 384;
          top_line = false;
        }

        // genug material für eine Zeile
        if (print_index >= 384 && !top_line) {
          // ESC * m nL nH
          //Serial.printf("start printing\n");
          if (!NO_PRINTER_MODE) {
            printer.write(27);
            printer.write('@');  // ESC @ means reset
            printer.write(27);
            printer.write(55);   // ESC 7 configure heating parameters
            printer.write(1);    // n1 = 5   → heizpunkte, die gleichzeitig laufen dürfen
            printer.write(255);  // n2 = 240 → lange Heizzeit (langsam, aber dunkler)
            printer.write(255);  // n3 = 200 → große Pause zwischen Heizungen, Strom sinkt stark

            printer.write(27);
            printer.write('*');
            printer.write((uint8_t)1);    // Mode 1 = 8-dot double density (384 pixel)
            printer.write((uint8_t)128);  // nL = 128 Bits
            printer.write((uint8_t)1);    // nH = 1 (1*256)    Zeilenlänge = nL + nH * 256 = 384

            // Daten senden
            for (int i = 0; i < 384; i++) {
              if ((i % 16) == 0)   // alle 16 Bytes
                vTaskDelay(1);
              printer.write(line_top[i] | line_bottom[i]);
              //if ((i & 0x1F) == 0)  // alle 32 Bytes, sonst evtl WDT fehler
                //vTaskDelay(1);      // oder taskYIELD();
            }
            //printer.write(10); // Zeilenvorschub (macht von alleine genau passend)
            //printer.flush();  // wartet, bis alles gesendet wurde, aber unnötig
            //Serial.printf("done printing\n");
            //vTaskDelay(1);
          }
          print_index = 0;
          top_line = true;
          something_in_it = false;
          memset(line_top, 0, sizeof(line_top));
          memset(line_bottom, 0, sizeof(line_bottom));
        }
        vTaskDelay(1);
      }
    }
    printHighwater = uxTaskGetStackHighWaterMark(NULL);
    taskYIELD();
  }
}