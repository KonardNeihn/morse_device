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

void showBadSignal() {
  for (int i = 0; i < 4; i++) {
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

void testMosfet() {
  digitalWrite(MOSFET, LOW);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  digitalWrite(MOSFET, HIGH);
}

bool connectWifi(const char ssid[], const char password[]) {
  disconnectTCP();
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  WiFi.mode(WIFI_STA);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);

  Serial.printf("Connecting to %s\n", ssid);

  WiFi.begin(ssid, password);

  if (WiFi.waitForConnectResult() == WL_CONNECTED) {
    Serial.println("WiFi OK");
    return true;
  }
  Serial.printf("Couldn't connect to %s\n", ssid);
  return false;
}

const char* disconnectReason(uint8_t reason) {
  switch (reason) {
    case WIFI_REASON_UNSPECIFIED: return "UNSPECIFIED";
    case WIFI_REASON_AUTH_EXPIRE: return "AUTH_EXPIRE";
    case WIFI_REASON_AUTH_LEAVE: return "AUTH_LEAVE";
    case WIFI_REASON_ASSOC_EXPIRE: return "ASSOC_EXPIRE";
    case WIFI_REASON_ASSOC_TOOMANY: return "ASSOC_TOOMANY";
    case WIFI_REASON_NOT_AUTHED: return "NOT_AUTHED";
    case WIFI_REASON_NOT_ASSOCED: return "NOT_ASSOCED";
    case WIFI_REASON_ASSOC_LEAVE: return "ASSOC_LEAVE";
    case WIFI_REASON_ASSOC_NOT_AUTHED: return "ASSOC_NOT_AUTHED";
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT: return "4WAY_HANDSHAKE_TIMEOUT";
    case WIFI_REASON_HANDSHAKE_TIMEOUT: return "HANDSHAKE_TIMEOUT";
    case WIFI_REASON_NO_AP_FOUND: return "NO_AP_FOUND";
    case WIFI_REASON_AUTH_FAIL: return "AUTH_FAIL";
    case WIFI_REASON_ASSOC_FAIL: return "ASSOC_FAIL";
    default: return "UNKNOWN";
  }
}

const char* wifiEventName(arduino_event_id_t event) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_READY: return "WIFI_READY";
    case ARDUINO_EVENT_WIFI_SCAN_DONE: return "SCAN_DONE";
    case ARDUINO_EVENT_WIFI_STA_START: return "STA_START";
    case ARDUINO_EVENT_WIFI_STA_STOP: return "STA_STOP";
    case ARDUINO_EVENT_WIFI_STA_CONNECTED: return "STA_CONNECTED";
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED: return "STA_DISCONNECTED";
    case ARDUINO_EVENT_WIFI_STA_GOT_IP: return "STA_GOT_IP";
    case ARDUINO_EVENT_WIFI_STA_LOST_IP: return "STA_LOST_IP";
    default: return "UNKNOWN";
  }
}

std::string packageToText(Package package) {
  std::string text = "";
  for (int i = 0; i < package.size; i++) {
    uint8_t mask = 0b10000000;
    for (int j = 0; j < 8; j++) {
      if (package.payload[i] & mask)
        text += "1";
      else
        text += "0";
      mask >>= 1;
    }
  }
  return text;
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

  package = *p;  // Deep Copy (std::vector kopiert korrekt)
  delete p;
  return true;
}

void disconnectTCP() {
  if (sock >= 0)
    close(sock);
  sock = -1;
  state = TCP_CONNECT;
}

bool connectTCP(int retry_counter) {
  Serial.println("Connecting TCP...");

  // socket erstellen
  sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);  // AF_INET → IPv4, SOCK_STREAM → TCP, IPPROTO_TCP → TCP-Protokoll
  if (sock < 0) {
    Serial.println("socket failed");
    return false;
  }

  sockaddr_in server_addr = { 0 };
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(port);
  server_addr.sin_addr.s_addr = (uint32_t)server_ip;
  //server_addr.sin_addr.s_addr = htonl((uint32_t)server_ip); wäre andere endianess. ist aber falsch

  if (connect(sock, (sockaddr*)&server_addr, sizeof(server_addr)) != 0) {
    Serial.printf("connect failed: %s (%d) (%d/%d)\n", strerror(errno), errno, retry_counter, MAX_CONNECT_RETRIES);

    disconnectTCP();
    return false;
  }

  Serial.printf("TCP connected \n");

  // keep alive aktivieren
  int yes = 1;
  setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &yes, sizeof(yes));
  // wie viele sekunden kein verkehr, bis erstes keep alive
  int idle = 10;
  setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
  // falls erstes keep alive keine antwort - wann das nächste in sekunden
  int interval = 5;
  setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &interval, sizeof(interval));
  // wie oft keep alive wiederholen
  int count = 3;
  setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &count, sizeof(count));

  timeval timeout;

  timeout.tv_sec = 5;   // in sekunden
  timeout.tv_usec = 0;  // in micro sekunden

  // send timeout
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  //receive timeout
  setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

  state = RUNNING;

  return true;
}

bool checkSocket(bool& readable, bool& writable) {
  // check if somethings wrong / client connected?
  int err;
  int len = sizeof(err);
  getsockopt(sock, SOL_SOCKET, SO_ERROR, &err, (socklen_t*)&len);

  if (err) {
    Serial.printf("Socket error: %s (%d)\n", strerror(err), err);
    disconnectTCP();
    return false;
  }

  if (sock < 0) {
    state = DNS_RESOLVE;
    Serial.printf("socket disconnected returning to DNS_RESOLVE\n");
    return false;
  }

  readable = false;
  writable = false;

  fd_set readfds;
  fd_set writefds;

  FD_ZERO(&readfds);
  FD_ZERO(&writefds);

  FD_SET(sock, &readfds);
  FD_SET(sock, &writefds);

  timeval tv{ 0, 0 };

  int ret = select(sock + 1,
                   &readfds,
                   &writefds,
                   nullptr,
                   &tv);

  if (ret < 0) {
    Serial.printf("select failed: %d\n", errno);
    disconnectTCP();
    return false;
  }

  readable = FD_ISSET(sock, &readfds);
  writable = FD_ISSET(sock, &writefds);

  return true;
}

bool sendAll(const uint8_t* data, size_t len) {
  size_t sent = 0;

  while (sent < len) {
    int n = send(sock, data + sent, len - sent, 0);
    if (n > 0) {
      sent += n;
    } else if (n == 0) {
      Serial.println("peer disconnected");
      disconnectTCP();
      return false;
    } else {
      Serial.printf("send failed: %d\n", errno);
      disconnectTCP();
      return false;
    }
    // errno == ETIMEDOUT -> Receive-Timeout
    // errno == ECONNRESET-> Gegenstelle hat die Verbindung zurückgesetzt.
    // errno == ENOTCONN  -> Socket ist nicht verbunden.
    // errno == EAGAIN    -> Timeout bei einem nicht blockierenden Socket oder wenn SO_RCVTIMEO abgelaufen ist (je nach Plattform kann auch EWOULDBLOCK verwendet werden).
  }
  return true;
}

bool recvAll(uint8_t* bytes, size_t bytesToRead) {
  size_t got = 0;

  while (got < bytesToRead) {
    int n = recv(sock, bytes + got, bytesToRead - got, MSG_WAITALL);

    if (n > 0) {
      got += n;
    } else if (n == 0) {
      Serial.println("peer disconnected");
      disconnectTCP();
      return false;
    } else {
      Serial.printf("recv failed: %d\n", errno);
      disconnectTCP();
      return false;
    }
    // errno == ETIMEDOUT -> Receive-Timeout
    // errno == ECONNRESET-> Gegenstelle hat die Verbindung zurückgesetzt.
    // errno == ENOTCONN  -> Socket ist nicht verbunden.
    // errno == EAGAIN    -> Timeout bei einem nicht blockierenden Socket oder wenn SO_RCVTIMEO abgelaufen ist (je nach Plattform kann auch EWOULDBLOCK verwendet werden).
  }

  return true;
}

void receivePackage() {
  Package incoming;

  // recv header
  uint8_t header[3];
  if (!recvAll(header, 3))
    return;

  incoming.status = header[0];
  incoming.size = (uint16_t(header[1]) << 8) | header[2];

  // recv payload
  incoming.payload.resize(incoming.size);
  if (!recvAll(incoming.payload.data(), incoming.size))
    return;

  Serial.printf("%s\n", packageToText(incoming).c_str());

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
  sendAll(packet.data(), packet.size());
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
