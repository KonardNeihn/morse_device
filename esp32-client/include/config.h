#pragma once

// =====================================================================
// Zentrale Konfiguration des Morse-Clients.
// Alle "Schräubchen zum Drehen" an einer Stelle.
// =====================================================================

// ------------------------- WLAN-Zugangsdaten -------------------------
constexpr const char* ssid = "GameOfWlan";
constexpr const char* password = "thenorthremembers";
constexpr const char* ssid2 = "Fairphone 6";
constexpr const char* password2 = "Hurensohn";

// ------------------------- Server-Konfiguration ----------------------
constexpr const char* server_address = "morse.ddns.berlin";  // Adresse des Servers
constexpr int port = 6969;                                   // Port des Servers (Senden)

// ------------------------- Timing -------------------------------------
#define SAMPLING_RATE_MS 10       // eine Abtastung alle x ms
#define RECORDING_TIMEOUT_MS 5000
#define TCP_TIMEOUT 30000
#define KEEP_ALIVE_INTERVAL_MS 5000
#define MAX_CONNECT_RETRIES 3
#define QUEUE_SIZE 10
#define SOUND_FREQ 200

// ------------------------- Pins ----------------------------------------
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
