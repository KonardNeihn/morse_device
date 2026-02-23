void playTone() {
  if (!NO_SOUND_MODE)          // nur wenn der drehschalter nicht "ohne Ton" sagt (LOW = angeschaltet)
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

// loopt solange, bis eins von zwei wlans verbunden ist
void checkWiFi() {
  while (WiFi.status() != WL_CONNECTED) {
    if (digitalRead(NO_SOUND_MODE_PIN) == LOW)
      NO_SOUND_MODE = true;
    connectWiFi(ssid, password);
    if (WiFi.status() != WL_CONNECTED)
      connectWiFi(ssid2, password2);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// versucht sich mit einem WLAN zu verbinden
void connectWiFi(const char *ssid, const char *password) {
  int connect_attempt = 0;  // verbindungs timeout
  //Serial.print("Verbinde mit ");
  //Serial.print(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED && connect_attempt <= 5) {
    beepOnce();
    //Serial.print(".");
    connect_attempt++;
  }
  //Serial.println("");
  if (WiFi.status() != WL_CONNECTED) {
    //Serial.println("Couldn't connect!");
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);  //sonst kann keine neue ssid & passwd vergeben werden
    vTaskDelay(500 / portTICK_PERIOD_MS);
    WiFi.mode(WIFI_STA);
  } else {
    //Serial.println("\nWLAN Verbunden!");
    //Serial.printf("IP-Adresse: %d.%d.%d.%d Empfang: %ddb\n", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3], WiFi.RSSI());
    beepTwice();
    WiFi.setSleep(false);  //um hoffentlich package loss zu verhindern
    udp.stop();
    udp.begin(udp_port);
    //Serial.printf("Lausche und sende vermutlich nicht auf Port: %d haha\n", udp_port);
  }
}

void send_and_shift_window(WindowSlot *sliding_window) {
  if (xQueueSend(playbackQueue, &sliding_window[0].event, 0) != pdPASS) {
    //Serial.printf("playbackQueue pass!!! \n");
  }
  if (xQueueSend(printQueue, &sliding_window[0].event, 0) != pdPASS) {
    //Serial.printf("printQueue pass!!! \n");
  }
  for (int i = 0; i < WINDOW_SIZE - 1; i++) {
    sliding_window[i] = sliding_window[i+1];
  }
  sliding_window[WINDOW_SIZE - 1].valid = false;
}

void playback(MorseEvent *event, bool *sound_on) {
  if ((event->state) == true) {     // equivalent mit (*event).state
    if (*sound_on == false) { // sonst hackt das bei dauerhaftem drücken
      playTone();
    }
    digitalWrite(LED, HIGH);
    *sound_on = true;
  } else {
    noTone(SPEAKER);
    digitalWrite(LED, LOW);
    *sound_on = false;
  }
  vTaskDelay((event->duration_ms) / portTICK_PERIOD_MS);
}

void print(bool top_line[384], bool bottom_line[384]) {
  // Reset with ESC @
  printer.write(27);  // ESC
  printer.write('@'); // @
  vTaskDelay(1);

  // configure heating parameters ESC 7
  printer.write(27);    // ESC
  printer.write(55);    // 7
  printer.write(1);     // n1   → heizpunkte, die gleichzeitig laufen dürfen              evtl dunkler bei größerer anzahl, das heizpunkte gemeinsam sich gegenseitig heizen
  printer.write(255);   // n2   → Länge Heizzeit (hoch -> langsam, aber dunkler)          evtl bringt 255 nicht so viel, 
  printer.write(255);   // n3   → Pause zwischen Heizungen (hoch -> Strom sinkt stark)    evtl hellere schrift bei mehr zeit, da benachbarte punkte sich gegenseitig vorwärmen (-> wartezeit abkühlen)
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

  for(int i = 0; i < 384; i++) {
    uint8_t column = 0b00010000;  // eine Trennlinie zwischen den Zeilen
    if(top_line[i])
      column = column | 0b11000000; // obere Zeile
    if(bottom_line[i])
      column = column | 0b00000110; // untere Zeile

    printer.write(column);  // die Spalte an den Drucker senden

    if(i % 16 == 0) // alle 16 Spalten mal kurz durchatmen (evtl auch wichtig für den watchdog)
      vTaskDelay(1);
  }
  printer.flush();
  vTaskDelay(50 / portTICK_PERIOD_MS);
}
