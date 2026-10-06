#pragma once

// =============================================================================
// config.h  –  Zentrale Konfiguration des Morse-Clients
//
// Alle "Schräubchen zum Drehen" an EINER Stelle. Wenn du z. B. die Tonhöhe
// oder den Namen des Konfigurations-Hotspots ändern willst, musst du nur
// diese Datei anfassen.
// =============================================================================

// ------------------------- Hotspot (Konfigurations-AP) -----------------
// Offener Access Point, über den man per Handy-Browser die WLANs einstellt.
// Die WLAN-Zugangsdaten selbst werden persistent im NVS gespeichert
// (siehe wifi_config.h/.cpp) und über die Portal-Webseite konfiguriert.
#define AP_SSID "Morse"   // Name des offenen Hotspots
#define AP_CHANNEL 1      // Funkkanal des Hotspots
#define AP_MAX_CONN 4     // max. gleichzeitig verbundene Clients

// ------------------------- Server-Konfiguration ----------------------
constexpr const char* server_address = "morse-server.de";  // Hostname des Servers
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
// GPIO-Nummern der angeschlossenen Hardware (ESP32-C5-DevKitC-1).
// Hinweis: Der ESP32-C5 hat nur GPIO0..GPIO28. Flash/PSRAM belegt GPIO16..22,
// Strapping-Pins sind 2/3/7/25/26/27/28, USB-JTAG 13/14, Konsole UART0 11/12.
#define TX_PIN 4                  // UART1 TX (zum Thermodrucker)
#define RX_PIN 5                  // UART1 RX (vom Thermodrucker)
#define SPEAKER 8                 // Lautsprecher (PWM über LEDC)
#define BUTTON 9                  // Morse-Taste (LOW = gedrückt)
#define LED 10                    // Status-LED
#define MOSFET 23                 // hält den ESP32 mit Strom (Selbsthalte-Schaltung)

// ------------------------- Drehschalter (6 Positionen über ADC) ---------
// Statt 6 einzelner GPIOs wird EIN ADC-Kanal gelesen. Ein Spannungsteiler
// (6 Widerstände 1 kOhm in Reihe zwischen 3V3 und GND) liefert an seinen
// 6 Abgriffen klar getrennte Spannungen; der Schleifer des Drehschalters legt
// einen Abgriff auf den ADC-Pin (GPIO1).
//
// Abgriff-Spannungen: 0 / 0,55 / 1,10 / 1,65 / 2,20 / 2,75 V
// (≈ ADC-Rohwerte 0 / 726 / 1453 / 2180 / 2907 / 3634 bei 12 Bit).
#define ROTARY_ADC_CHANNEL ADC_CHANNEL_0  // ADC1_CH0 = GPIO1 am DevKitC-1
#define ROTARY_ATTEN      ADC_ATTEN_DB_12 // Messbereich ~0..3,1 V

// Schwellwerte = Mitten zwischen zwei benachbarten Abgriffen. Reihenfolge
// (von niedrig nach hoch): NORMAL -> NO_SOUND -> NO_PRINTER -> SELF_CHECK ->
// SERVER_CHECK -> HOTSPOT. Bei anderer Verdrahtung hier umsortieren.
#define ROTARY_TH_1  363   // zwischen NORMAL     und NO_SOUND
#define ROTARY_TH_2 1090   // zwischen NO_SOUND   und NO_PRINTER
#define ROTARY_TH_3 1817   // zwischen NO_PRINTER und SELF_CHECK
#define ROTARY_TH_4 2543   // zwischen SELF_CHECK und SERVER_CHECK
#define ROTARY_TH_5 3271   // zwischen SERVER_CHECK und HOTSPOT

// ------------------------- Drucker-Timing (Supercap) ----------------------
// Der Thermodrucker hat einen 3F-Supercap, der die hohen Stromspitzen der
// Heizzeile puffert und mit begrenztem Strom auflädt (~30 s bis voll). Wird
// gedruckt, bevor er geladen ist (oder bevor er zwischen zwei Zeilen wieder
// nachgeladen hat), ist der Abdruck zu blass/unleserlich.
//
// Einfacher Ansatz statt Spannungsmessung: Nach dem Booten einmal 30 s warten
// (Erstaufladung), und zwischen zwei Druckzeilen 5 s nachladen.
#define PRINT_BOOT_DELAY_MS 30000  // Wartezeit nach dem Booten vor dem ersten Druck
#define PRINT_LINE_DELAY_MS 5000   // Wartezeit zwischen zwei Druckzeilen


