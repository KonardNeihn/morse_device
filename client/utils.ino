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

char *signalToText(uint8_t signal) {
  static char text[9];
  uint8_t mask = 0b10000000;

  for (int j = 0; j < 8; j++) {
    if (signal & mask)
      text[j] = '1';
    else
      text[j] = '0';
    mask >>= 1;
  }
  text[8] = '\0';
  return text;
}

void testMosfet() {
  digitalWrite(MOSFET, LOW);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  digitalWrite(MOSFET, HIGH);
}

void handlePackets() {

  if (!client.connected()) {
    Serial.println("VERBINDUNG ABGEBROCHEN: Vor dem Empfangen!");
    state = TCP_CONNECT;
    return;
  }

  // =========================
  // 1) RX IMMER ZUERST
  // =========================

  // read header
  if (receiveState == WAIT_FOR_HEADER) {
    uint8_t header[3];
    if (client.available() >= sizeof(header)) {
      size_t bytesSent = client.read(header, sizeof(header));
      if (bytesSent != sizeof(header)) {
        Serial.printf("FEHLER: Nur %d von %d Bytes empfangen!\n", bytesSent, sizeof(header));
        client.stop();
        state = TCP_CONNECT;
        return;
      }
      bytes_to_read = header[1];
      bytes_to_read << 8;
      bytes_to_read += header[2];
      receiveState = WAIT_FOR_PAYLOAD;
    }
  }

  // read payload
  while (receiveState == WAIT_FOR_PAYLOAD && client.available() >= 1) {
    uint8_t signal;
    size_t bytesSent = client.read(&signal, sizeof(signal));
    if (bytesSent != sizeof(signal)) {
      Serial.printf("FEHLER: Nur %d von %d Bytes empfangen!\n", bytesSent, sizeof(signal));
      client.stop();
      state = TCP_CONNECT;
      return;
    }

    bytes_to_read--;
    Serial.printf(signalToText(signal));
    if (bytes_to_read == 0) {
      receiveState = WAIT_FOR_HEADER;
      Serial.printf("\n");
    }

    while (xQueueSend(printQueue, &signal, SAMPLING_RATE_MS) != pdPASS)
      Serial.printf("printQueue overflow!!!\n");

    while (xQueueSend(playbackQueue, &signal, SAMPLING_RATE_MS) != pdPASS)
      Serial.printf("playbackQueue overflow!!!\n");
  }

  if (!client.connected()) {
    Serial.println("VERBINDUNG ABGEBROCHEN: Nach dem Empfangen!");
    state = TCP_CONNECT;
    return;
  }

  // =========================
  // 2) SAMMLE DATEN, SOLANGE AKTIV
  // =========================

  uint8_t outgoing;
  while (xQueueReceive(sendQueue, &outgoing, 0) == pdPASS) {
    outgoing_signal.push_back(outgoing);
    Serial.printf("%d \n", outgoing_signal.size());
  }

  // =========================
  // 3) WENN INAKTIV -> SENDEN
  // =========================

  if (!client.connected()) {
    Serial.println("VERBINDUNG ABGEBROCHEN: Vor dem Senden!");
    state = TCP_CONNECT;
    return;
  }

  if ((millis() - last_own_activity > INACTIVITY_TIMEOUT_MS + 500) && outgoing_signal.size() > 0) { // +500 damit die queue auch wirklich leer ist
    Serial.printf("last_own_activity: %d\n", millis() - last_own_activity);
    Serial.printf("dynamic buffer size: %d\n", outgoing_signal.size());

    // Create & Send Header = status + length
    uint8_t header[3];
    uint8_t status = SERVER_CHECK_MODE ? 1 : 0;
    uint16_t size = static_cast<uint16_t>(outgoing_signal.size());
    header[0] = status;
    header[1] = (size >> 8) & 0xFF;   // Hochwertiges Byte von size
    header[2] = size & 0xFF;          // Niedrigwertiges Byte von size
    client.write(header, sizeof(header));
    size_t bytesSent = client.write(header, sizeof(header));
    if (bytesSent != sizeof(header)) {
      Serial.printf("FEHLER: Nur %d von %d Bytes gesendet!\n", bytesSent, sizeof(header));
      client.stop();
      state = TCP_CONNECT;
      return;
    }
    Serial.printf("%s \n", signalToText(header[0]));
    Serial.printf("%s \n", signalToText(header[1]));
    Serial.printf("%s \n", signalToText(header[2]));

    // Send signal
    bytesSent = client.write(outgoing_signal.data(), outgoing_signal.size());
    if (bytesSent != outgoing_signal.size()) {
      Serial.printf("FEHLER: Nur %d von %d Bytes gesendet!\n", bytesSent, outgoing_signal.size());
      client.stop();
      state = TCP_CONNECT;
      return;
    }

    outgoing_signal.clear();
  }

  if (!client.connected()) {
    Serial.println("VERBINDUNG ABGEBROCHEN: Nach dem Senden!");
    state = TCP_CONNECT;
    return;
  }
}

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

    // zurücksetzen
    top_line[i] = false;
    bottom_line[i] = false;

    if (i % 16 == 0)  // alle 16 Spalten mal kurz durchatmen (evtl auch wichtig für den watchdog)
      vTaskDelay(1);
  }
  printer.flush();
  vTaskDelay(50 / portTICK_PERIOD_MS);
}
