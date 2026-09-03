#pragma once

#include <cstddef>
#include <cstdint>

#include <WiFi.h>  // arduino_event_id_t

// ------------------------- WLAN ---------------------------------------
bool connectWifi(const char ssid[], const char password[]);

// Debug-Helfer für WiFi-Events.
const char* disconnectReason(uint8_t reason);
const char* wifiEventName(arduino_event_id_t event);

// ------------------------- DNS + TCP ----------------------------------
bool resolveDNS();
bool connectTCP(int retry_counter);
void disconnectTCP();
bool checkSocket(bool& readable, bool& writable);

// ------------------------- Senden / Empfangen -------------------------
bool sendAll(const uint8_t* data, size_t len);
bool recvAll(uint8_t* bytes, size_t bytesToRead);
void receivePackage();
void sendPackage();
