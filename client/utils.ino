void showSearchingWiFi() {
  for (int i = 0; i < 1; i++) {
    digitalWrite(LED, HIGH);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    digitalWrite(LED, LOW);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void showResolvingDNS() {
  for (int i = 0; i < 2; i++) {
    digitalWrite(LED, HIGH);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    digitalWrite(LED, LOW);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void showConnectTCP() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED, HIGH);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    digitalWrite(LED, LOW);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }

void hearingNothing() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(LED, HIGH);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    digitalWrite(LED, LOW);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void playTone() {
  if (!NO_SOUND_MODE)           // nur wenn der drehschalter nicht "ohne Ton" sagt (LOW = angeschaltet)
    tone(SPEAKER, SOUND_FREQ);  // tone() blockiert auf dem esp32 den thread NICHT
}

void checkPins() {
  NO_SOUND_MODE = (digitalRead(NO_SOUND_MODE_PIN) == LOW);
  NO_PRINTER_MODE = (digitalRead(NO_PRINTER_MODE_PIN) == LOW);
  SELF_CHECK_MODE = (digitalRead(SELF_CHECK_MODE_PIN) == LOW);
  SERVER_CHECK_MODE = (digitalRead(SERVER_CHECK_MODE_PIN) == LOW);
  RICK_ROLL_MODE = (digitalRead(RICK_ROLL_MODE_PIN) == LOW);
}

char *signalToText(uint8_t signal[FRAMES_PER_PACKET]) {
  static char text[(FRAMES_PER_PACKET * 8) + 1];
  // für jedes byte
  for (int i = 0; i < FRAMES_PER_PACKET; i++) {

    uint8_t mask = 0b10000000;
    for (int j = 0; j < 8; j++) {
      if (signal[i] & mask)
        text[(i*8)+j] = '1';
      else
        text[(i*8)+j] = '0';
      mask >>= 1;
    }
  }
  text[FRAMES_PER_PACKET * 8] = '\0';
  return text;
}

void testMosfet() {
  digitalWrite(MOSFET, LOW);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  digitalWrite(MOSFET, HIGH);
}

void playback(uint8_t *signal, bool *sound_on) {
  // für jedes byte
  for (int i = 0; i < FRAMES_PER_PACKET; i++) {

    uint8_t mask = 0b10000000;

    for (int j = 0; j < 8; j++) {
      if ((signal[i] & mask)) {
        if (*sound_on == false) {
          playTone();
          digitalWrite(LED, HIGH);
          *sound_on = true;
        }
      } else {
        noTone(SPEAKER);
        digitalWrite(LED, LOW);
        *sound_on = false;
      }
      mask >>= 1;
      vTaskDelay(SAMPLING_RATE_MS / portTICK_PERIOD_MS);
    }
  }
}

void handlePackets() {
  Packet outgoing;
  Packet incoming;

  outgoing.status = SERVER_CHECK_MODE ? 1 : 0;

  // =========================
  // 1) RX IMMER ZUERST
  // =========================
  while (client.connected() && client.available() >= sizeof(Packet)) {
    int n = client.read((uint8_t *)&incoming, sizeof(Packet));
    if (n != sizeof(Packet)) {
      Serial.printf("TCP read failed: got %d of %d bytes\n", n, sizeof(Packet));
      client.stop();
      state = TCP_CONNECT;
      return;
    }

    /*if (incoming.status == 2) {
      Serial.printf("ping\n");
      last_ping = millis();
      continue;
    }*/

    last_rx = millis();

    if (xQueueSend(printQueue, &incoming.signal, 0) != pdPASS)
      Serial.printf("printQueue overflow!!!\n");

    if (xQueueSend(playbackQueue, &incoming.signal, 0) != pdPASS)
      Serial.printf("playbackQueue overflow!!!\n");
  }

  // =========================
  // 2) GENAU EIN TX-VERSUCH
  // =========================
  if (xQueuePeek(sendQueue, &outgoing.signal, 0) == pdPASS && client.connected()) {

    uint32_t t0 = millis();
    int written = client.write((uint8_t *)&outgoing, sizeof(Packet));
    uint32_t dt = millis() - t0;

    if (dt > 20) {
      Serial.printf("TX slow: %ums\n", (unsigned)dt);
    }

    /*if (dt > 200) {
      Serial.printf("TX STALL %ums -> reconnect\n", (unsigned)dt);
      client.stop();
      state = TCP_CONNECT;
      return;
    }*/

    if (written == sizeof(Packet)) {
      // erst jetzt aus Queue entfernen
      xQueueReceive(sendQueue, &outgoing.signal, 0);
    } else {
      Serial.printf("TCP write failed: wrote %d of %d bytes\n", written, sizeof(Packet));
      client.stop();
      state = TCP_CONNECT;
      return;
    }
  }
}

/*
bool timedWrite(Packet* data) {
  size_t len = sizeof(Packet);
  uint32_t t0 = millis();
  Serial.printf("[TX] write enter len=%u availForWrite=%d connected=%d\n",
                (unsigned)len,
                client.availableForWrite(),
                client.connected());

  size_t n = client.write((uint8_t*)data, len);

  uint32_t dt = millis() - t0;
  Serial.printf("[TX] write exit  n=%u dt=%ums availForWrite=%d connected=%d\n",
                (unsigned)n,
                (unsigned)dt,
                client.availableForWrite(),
                client.connected());

  if (dt > 50) {
    Serial.printf("!!! TX BLOCKED %ums !!!\n", (unsigned)dt);
  }

  return n == len;
}

bool timedReadPacket(Packet* pkt) {
  uint32_t t0 = millis();
  Serial.printf("[RX] read enter avail=%d connected=%d\n",
                client.available(),
                client.connected());

  int n = client.read((uint8_t*)pkt, sizeof(Packet));

  uint32_t dt = millis() - t0;
  Serial.printf("[RX] read exit  n=%d dt=%ums avail=%d connected=%d\n",
                n,
                (unsigned)dt,
                client.available(),
                client.connected());

  if (dt > 50) {
    Serial.printf("!!! RX BLOCKED %ums !!!\n", (unsigned)dt);
  }

  return n == sizeof(Packet);
}
*/

void print(bool top_line[384], bool bottom_line[384]) {
  // Reset with ESC @
  printer.write(27);   // ESC
  printer.write('@');  // @
  vTaskDelay(1);

  // configure heating parameters ESC 7
  printer.write(27);   // ESC
  printer.write(55);   // 7
  printer.write(1);    // n1   → heizpunkte, die gleichzeitig laufen dürfen              evtl dunkler bei größerer anzahl, das heizpunkte gemeinsam sich gegenseitig heizen
  printer.write(255);  // n2   → Länge Heizzeit (hoch -> langsam, aber dunkler)          evtl bringt 255 nicht so viel,
  printer.write(255);  // n3   → Pause zwischen Heizungen (hoch -> Strom sinkt stark)    evtl hellere schrift bei mehr zeit, da benachbarte punkte sich gegenseitig vorwärmen (-> wartezeit abkühlen)
  vTaskDelay(1);

  // Größe des Bildes
  printer.write(27);
  printer.write('*');
  printer.write((uint8_t)1);    // Mode 1 = 8-dot double density (384 pixel)
  printer.write((uint8_t)128);  // nL = 128 Bits
  printer.write((uint8_t)1);    // nH = 1 (1*256)    Zeilenlänge = nL + nH * 256 = 384
  vTaskDelay(1);

  for (int i = 0; i < 384; i++) {
    uint8_t column = 0b00010000;  // eine Trennlinie zwischen den Zeilen
    if (top_line[i])
      column = column | 0b11000000;  // obere Zeile
    if (bottom_line[i])
      column = column | 0b00000110;  // untere Zeile

    printer.write(column);  // die Spalte an den Drucker senden

    if (i % 16 == 0)  // alle 16 Spalten mal kurz durchatmen (evtl auch wichtig für den watchdog)
      vTaskDelay(1);
  }
  printer.flush();
  vTaskDelay(50 / portTICK_PERIOD_MS);
}
