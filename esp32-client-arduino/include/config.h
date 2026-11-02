#pragma once

// =============================================================================
// config.h  –  Zentrale Konfiguration des Morse-Clients
//
// Alle "Schräubchen zum Drehen" an EINER Stelle. Wenn du z. B. die Tonhöhe
// oder den Namen des Konfigurations-Hotspots ändern willst, musst du nur
// diese Datei anpassen.
// =============================================================================

// ------------------------- Firmware-Version -----------------------------
// 1.0.0 war erste arduino version
// 2.0.0 war dann esp-idf
// 3.0.0 ist wieder arduino framework
#define FIRMWARE_VERSION "3.0.0"

// ------------------------- Logging -------------------------------------
// 0 = NONE, 1 = ERROR, 2 = WARN, 3 = INFO, 4 = DEBUG, 5 = VERBOSE
#define LOG_LEVEL 3        // normal: INFO; beim Debuggen auf 4/5 stellen


// ------------------------- Hotspot (Konfigurations-AP) -----------------
// Offener Access Point, über den man per Handy-Browser die WLANs einstellt.
// Die WLAN-Zugangsdaten selbst werden persistent im NVS gespeichert
// (siehe storage.h/.cpp) und über die Portal-Webseite konfiguriert.
#define AP_SSID "Morse"   // Name des offenen Hotspots
#define AP_CHANNEL 1      // Funkkanal des Hotspots
//#define AP_MAX_CONN 4     // max. gleichzeitig verbundene Clients

// ------------------------- Server-Konfiguration ----------------------
constexpr const char* server_address = "morse-server.de";  // Hostname des Servers
constexpr int port = 6969;                                   // Port des Servers (Senden)

// ------------------------- Timing -------------------------------------
// Alle Zeiten in Millisekunden, sofern nicht anders angegeben.
#define SAMPLING_RATE_MS 10       // eine Abtastung der Taste alle x ms
#define RECORDING_TIMEOUT_MS 5000 // nach so langer Pause gilt die Morse-Eingabe als fertig
#define TCP_KEEPALIVE_IDLE_S     10  // Sekunden Inaktivität bis zur 1. Keepalive-Probe
#define TCP_KEEPALIVE_INTERVAL_S  5  // Abstand zwischen den Keepalive-Proben
#define TCP_KEEPALIVE_COUNT       3  // erfolglose Proben, bis die Verbindung als tot gilt
#define MAX_CONNECT_RETRIES 3     // max. DNS-/TCP-Versuche, bevor ein Zustand zurückfällt
#define QUEUE_SIZE 10             // Pufferplätze der Nachrichten-Queues
#define SOUND_FREQ 700            // Tonhöhe des Lautsprechers in Hz

// ------------------------- Pins ----------------------------------------
// GPIO-Nummern der angeschlossenen Hardware (ESP32-C5-DevKitC-1).
// Hinweis: Strapping-Pins sind 2/3/7/25/26/27/28, USB-JTAG 13/14,
// Konsole UART0 11/12. Das SPI-PSRAM belegt GPIO16..22 UND GPIO15 (SPICS1) –
// GPIO15 ist auf diesem Board (N8R4 mit PSRAM) deshalb nicht nutzbar.
#define TX_PIN 4                  // UART1 TX (zum Thermodrucker)
#define RX_PIN 5                  // UART1 RX (vom Thermodrucker)
#define PIEZO_PIN 24                 // Lautsprecher (PWM über LEDC)
#define BUTTON_PIN 9                  // Morse-Taste (LOW = gedrückt)
#define LED_PIN 10                    // Status-LED
#define MOSFET_PIN 23                 // lädt den Drucker bzw. kappt den ESP32 vom Supercap
#define AUDIO_MODE_PIN 0             // Kippschalter 
#define PRINTER_MODE_PIN 1           // Kippschalter 
#define HOTSPOT_MODE_PIN 6           // Kippschalter            
#define TEST_MODE_PIN 8              // Kippschalter 

// ------------------------- Drucker-Timing (Supercap) ----------------------
// Der Thermodrucker hat einen 3F-Supercap, der die hohen Stromspitzen der
// Heizzeile puffert und mit begrenztem Strom auflädt (~30 s bis voll). Wird
// gedruckt, bevor er geladen ist (oder bevor er zwischen zwei Zeilen wieder
// nachgeladen hat), ist der Abdruck zu blass/unleserlich.
//
// Einfacher Ansatz statt Spannungsmessung: Nach dem Booten einmal 60 s warten
// (Erstaufladung), und zwischen zwei Druckzeilen 5 s nachladen.
#define PRINT_BOOT_DELAY_MS 60000  // Wartezeit nach dem Booten vor dem ersten Druck
#define PRINT_LINE_DELAY_MS 5000   // Wartezeit zwischen zwei Druckzeilen


