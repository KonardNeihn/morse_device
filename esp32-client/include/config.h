#pragma once

// =============================================================================
// config.h  –  Zentrale Konfiguration des Morse-Clients
//
// Alle "Schräubchen zum Drehen" an EINER Stelle. Wenn du z. B. WLAN-Daten
// oder die Tonhöhe ändern willst, musst du nur diese Datei anfassen.
// =============================================================================

// ------------------------- WLAN-Zugangsdaten -------------------------
// Erstes (primäres) WLAN und zweites (Fallback-) WLAN.
constexpr const char* ssid = "GameOfWlan";
constexpr const char* password = "thenorthremembers";
constexpr const char* ssid2 = "Fairphone 6";
constexpr const char* password2 = "Hurensohn";

// ------------------------- Server-Konfiguration ----------------------
constexpr const char* server_address = "morse.ddns.berlin";  // Hostname des Servers
constexpr int port = 6969;                                   // Port des Servers (Senden)

// ------------------------- Timing -------------------------------------
// Alle Zeiten in Millisekunden, sofern nicht anders angegeben.
#define SAMPLING_RATE_MS 10       // eine Abtastung der Taste alle x ms
#define RECORDING_TIMEOUT_MS 5000 // nach so langer Pause gilt die Morse-Eingabe als fertig
#define TCP_TIMEOUT 30000         // Timeout für TCP-Operationen
#define KEEP_ALIVE_INTERVAL_MS 5000
#define MAX_CONNECT_RETRIES 3     // max. DNS-/TCP-Versuche, bevor ein Zustand zurückfällt
#define QUEUE_SIZE 10             // Pufferplätze der Nachrichten-Queues
#define SOUND_FREQ 200            // Tonhöhe des Lautsprechers in Hz

// ------------------------- Pins ----------------------------------------
// GPIO-Nummern der angeschlossenen Hardware.
#define TX_PIN 17                 // UART TX (zum Thermodrucker)
#define RX_PIN 16                 // UART RX (vom Thermodrucker)
#define SPEAKER 12                // Lautsprecher (PWM über LEDC)
#define BUTTON 14                 // Morse-Taste (LOW = gedrückt)
#define LED 13                    // Status-LED
#define MOSFET 18                 // hält den ESP32 mit Strom (Selbsthalte-Schaltung)
#define NORMAL_MODE_PIN 27        // Drehschalter: normaler Betrieb
#define NO_SOUND_MODE_PIN 26      // Drehschalter: ohne Ton
#define NO_PRINTER_MODE_PIN 25    // Drehschalter: ohne Drucker
#define SELF_CHECK_MODE_PIN 33    // Drehschalter: Selbsttest
#define SERVER_CHECK_MODE_PIN 32  // Drehschalter: Server-Check
#define RICK_ROLL_MODE_PIN 35     // Drehschalter: Rick Roll

