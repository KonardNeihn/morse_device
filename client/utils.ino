void showSearchingWiFi() {
  for (int i = 0; i < 1; i++) {
    digitalWrite(LED, HIGH);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    digitalWrite(LED, LOW);
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

void showResolvingDNS() {
  for (int i = 0; i < 2; i++) {
    digitalWrite(LED, HIGH);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    digitalWrite(LED, LOW);
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

void showConnectTCP() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED, HIGH);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    digitalWrite(LED, LOW);
    vTaskDelay(200 / portTICK_PERIOD_MS);
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

void connectWifi(const char ssid[], const char password[]) {
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);

  Serial.printf("Connecting to %s\n", ssid);

  WiFi.begin(ssid, password);

  if (WiFi.waitForConnectResult() == WL_CONNECTED) {
    state = DNS_RESOLVE;
    Serial.println("WiFi OK");
  } 
}

// ist nötig, um std::vector<> senden zu können (liegt im heap)
bool putPackageIntoQueue(QueueHandle_t queue, const Package& package) {
  Package* copy = new Package(package);
  if (xQueueSend(queue, &copy, 0) != pdPASS) {
    Serial.printf("Queue overflow!!!\n");
    delete copy;
    return false;
  }
  return true;
}

// ist nötig, um std::vector<> senden zu können (liegt im heap)
bool getPackageFromQueue(QueueHandle_t queue, Package& package) {
  Package* p;
  if (xQueueReceive(queue, &p, 0) != pdPASS)
    return false;

  package = *p;   // Deep Copy (std::vector kopiert korrekt)
  delete p;
  return true;
}

void receivePackage(unsigned long *last_received) {
  // neues packet?
  if (!client.available())
    return;

  Package incoming;
  uint16_t bytes_to_read;

  // read status
  size_t bytesRead = client.read(&incoming.status, sizeof(incoming.status));
  if (bytesRead != sizeof(incoming.status)) {
    Serial.printf("FEHLER: Nur %d von %d Bytes von Package.status empfangen!\n", bytesRead, sizeof(incoming.status));
    client.stop();
    state = DNS_RESOLVE;
    return;
  }
  *last_received = millis();

  // wait for Package.size
  while(client.available() < sizeof(incoming.size)) {
    vTaskDelay(10 / portTICK_PERIOD_MS);
    if (millis() - *last_received > TCP_TIMEOUT) {
      Serial.printf("FEHLER: TCP Timeout beim empfangen von Package.size\n");
      client.stop();
      state = DNS_RESOLVE;
      return;
    }
  }

  // read size
  uint8_t size[2];
  bytesRead = client.read(size, sizeof(size));
  if (bytesRead != sizeof(size)) {
    Serial.printf("FEHLER: Nur %d von %d Bytes von Package.size empfangen!\n", bytesRead, sizeof(size));
    client.stop();
    state = DNS_RESOLVE;
    return;
  }
  incoming.size = (uint16_t(size[0]) << 8) | size[1];
  *last_received = millis();
  bytes_to_read = incoming.size;

  // read Package.payload
  while(bytes_to_read > 0) {
    if (client.available()) {
      uint8_t signal;
      bytesRead = client.read(&signal, sizeof(signal));
      if (bytesRead != sizeof(signal)) {
        Serial.printf("FEHLER: Nur %d von %d Bytes von Package.payload empfangen!\n", bytesRead, sizeof(signal));
        client.stop();
        state = DNS_RESOLVE;
        return;
      }
      *last_received = millis();
      bytes_to_read--;
      Serial.printf(signalToText(signal));
      incoming.payload.push_back(signal);
    } else {
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    if (millis() - *last_received > TCP_TIMEOUT) {
      Serial.printf("FEHLER: TCP Timeout beim empfangen von Package.size\n");
      client.stop();
      state = DNS_RESOLVE;
      return;
    }
  }
  Serial.printf("\n");

  if (incoming.status == 0) {
    Serial.printf("keep alive got\n");
    return;
  }

  if (!putPackageIntoQueue(playbackQueue, incoming))
    Serial.println("playbackQueue overflow");

  // confirmation package
  if (incoming.status == 2)
    return;

  // got my own server check message
  if (incoming.status == 3)
    return;

  // send receiving confirmation
  Package confirmation;
  confirmation.status = 2;
  confirmation.payload.push_back(0b11111111);
  confirmation.payload.push_back(0);
  confirmation.payload.push_back(0b11111111);
  confirmation.payload.push_back(0b11111111);
  confirmation.payload.push_back(0b11111111);
  confirmation.payload.push_back(0);
  confirmation.payload.push_back(0b11111111);
  confirmation.size = confirmation.payload.size();

  if (!putPackageIntoQueue(sendQueue, confirmation))
    Serial.println("sendQueue overflow");
}

void sendPackage() {
  Package outgoing;
  // package to send?
  if (uxQueueMessagesWaiting(sendQueue) == 0)
    return;
  getPackageFromQueue(sendQueue, outgoing);

  // gesamtes packet erzeugen
  std::vector<uint8_t> packet;
  // header anhängen
  packet.push_back(outgoing.status);
  packet.push_back((outgoing.size >> 8) & 0xFF);  // Hochwertiges Byte von size
  packet.push_back(outgoing.size & 0xFF);         // Niedrigwertiges Byte von size
  // payload anhängen
  packet.insert(packet.end(), outgoing.payload.begin(), outgoing.payload.end());

  // Send package
  size_t bytesSent = client.write(packet.data(), packet.size());
  if (bytesSent != packet.size()) {
    Serial.printf("FEHLER: Nur %d von %d Bytes gesendet!\n", bytesSent, packet.size());
    client.stop();
    state = DNS_RESOLVE;
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

    if (i % 16 == 0)  // alle 16 Spalten mal kurz durchatmen (evtl auch wichtig für den watchdog)
      vTaskDelay(1);
  }
  printer.flush();
  vTaskDelay(50 / portTICK_PERIOD_MS);
}
