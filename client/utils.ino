void playTone() {
  if (!NO_SOUND_MODE)           // nur wenn der drehschalter nicht "ohne Ton" sagt (LOW = angeschaltet)
    tone(SPEAKER, SOUND_FREQ);  // tone() blockiert auf dem esp32 den thread NICHT
}

void beepOnce() {
  playTone();
  digitalWrite(LED, HIGH);
  vTaskDelay(150 / portTICK_PERIOD_MS);
  noTone(SPEAKER);
  digitalWrite(LED, LOW);
  vTaskDelay(850 / portTICK_PERIOD_MS);
}

void beepTwice() {
  playTone();
  digitalWrite(LED, HIGH);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  noTone(SPEAKER);
  digitalWrite(LED, LOW);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  playTone();
  digitalWrite(LED, HIGH);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  noTone(SPEAKER);
  digitalWrite(LED, LOW);
}

void checkWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect(true, true);
    delay(100);
    WiFi.mode(WIFI_STA);
  }
  if (WiFi.status() != WL_CONNECTED)
    connectWiFi(ssid, password);
  if (WiFi.status() != WL_CONNECTED)
    connectWiFi(ssid2, password2);
  //if (WiFi.status() != WL_CONNECTED)
  //  ESP.restart();
}

void checkPins() {
  NO_SOUND_MODE = (digitalRead(NO_SOUND_MODE_PIN) == LOW);
  NO_PRINTER_MODE = (digitalRead(NO_PRINTER_MODE_PIN) == LOW);
  SELF_CHECK_MODE = (digitalRead(SELF_CHECK_MODE_PIN) == LOW);
  SERVER_CHECK_MODE = (digitalRead(SERVER_CHECK_MODE_PIN) == LOW);
  RICK_ROLL_MODE = (digitalRead(RICK_ROLL_MODE_PIN) == LOW);
}

void testMosfet() {
  digitalWrite(MOSFET, LOW);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  digitalWrite(MOSFET, HIGH);
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
    testMosfet();  // contains 100ms pause
    checkPins();
    connect_attempt++;
  }
  Serial.println("");
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Couldn't connect!");
    WiFi.disconnect(true, true);  //sonst kann keine neue ssid & passwd vergeben werden
    delay(100);
    WiFi.mode(WIFI_STA);
  } else {
    Serial.println("WLAN Verbunden!");
    Serial.printf("IP-Adresse: %d.%d.%d.%d Empfang: %ddb\n", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3], WiFi.RSSI());
    beepTwice();
    WiFi.setSleep(false);  //um hoffentlich package loss zu verhindern
    esp_wifi_set_ps(WIFI_PS_NONE);
    udp.stop();
    udp.begin(udp_port);
    Serial.printf("Lausche und sende vermutlich nicht auf Port: %d haha\n", udp_port);
  }
}

void playback(uint32_t *signal, bool *sound_on) {
  uint32_t mask = 0b00000001;
  for (int i = 0; i < SAMPLES_PER_FRAME; i++) {
    if ((*signal & mask) && *sound_on == false) {
      playTone();
      digitalWrite(LED, HIGH);
      *sound_on = true;
    } else {
      noTone(SPEAKER);
      digitalWrite(LED, LOW);
      *sound_on = false;
    }
    mask <<= 1;
    vTaskDelay(SAMPLING_RATE_MS / portTICK_PERIOD_MS);
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

  // ESC 3 Line spacing auf 0 setzen
  //printer.write(27);
  //printer.write('3');
  //printer.write(0);

  // Fett
  //printer.write(27);
  //printer.write(69);

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
