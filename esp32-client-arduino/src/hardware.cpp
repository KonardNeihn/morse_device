// =============================================================================
// hardware.cpp  –  Initialisierung und Ansteuerung der Hardware
//
// Konfiguriert alle GPIOs und steuert die Peripherie an. Im Einzelnen:
//   - Ausgänge:  Status-LED, Piezo-Lautsprecher, MOSFET
//   - Eingänge:  Morse-Taste, vier Kippschalter (je mit Pull-up)
//   - Helfer:    LED-Blinkmuster, Ton an/aus, Kippschalter einlesen,
//                MOSFET-Selbsttest
// =============================================================================

#include "hardware.h"
#include "HardwareSerial.h"
#include "app.h"
#include "config.h"
#include "esp32-hal-gpio.h"
#include "log.h"

#include <Arduino.h>             // tone(), noTone(), pinMode(), digitalWrite(), ...
#include <atomic>
#include <freertos/task.h>       // vTaskDelay, pdMS_TO_TICKS

static const char* TAG = "hardware";

static bool last_audio_mode   = false;
static bool last_printer_mode = false;
static bool last_hotspot_mode = false;
static bool last_test_mode    = false;

static bool soundIsOn = false;          // merkt sich, ob gerade ein Ton läuft

void hardwareInit() {
    // --- Ausgänge: LED, Piezo, MOSFET. ---
    // MOSFET HIGH = Drucker-Supercap wird geladen; LOW = ESP wird vom
    // Supercap getrennt und geht bei Wegfall der Hauptversorgung sauber aus.
    pinMode(LED_PIN,    OUTPUT);
    pinMode(PIEZO_PIN,  OUTPUT);
    pinMode(MOSFET_PIN, OUTPUT);

    // --- Eingänge: Morse-Taste + vier Kippschalter. ---
    // Interner Pull-up = Ruhepegel HIGH, gedrückt/aktiv = LOW.
    pinMode(BUTTON_PIN,       INPUT_PULLUP);
    pinMode(AUDIO_MODE_PIN,   INPUT_PULLUP);
    pinMode(PRINTER_MODE_PIN, INPUT_PULLUP);
    pinMode(HOTSPOT_MODE_PIN, INPUT_PULLUP);
    pinMode(TEST_MODE_PIN,    INPUT_PULLUP);

    // MOSFET einschalten: Supercap lädt, Drucker wird mit Strom versorgt.
    digitalWrite(MOSFET_PIN, HIGH);

    LOGI(TAG, "GPIOs initialisiert");
}

// ------------------------- LED-Blinkmuster --------------------------------
// Ein einzelner Blink: LED an, warten, LED aus, warten.
// vTaskDelay() statt delay(), weil diese Funktion aus FreeRTOS-Tasks
// heraus aufgerufen wird (CheckerTask) – dort ist delay() unerwünscht.
static void blink(int delay_ms) {
    // während der Nutzer morst oder nachricht bekommt -> keine Status-LED.
    if (isPlaying || isRecording)
        return;

    digitalWrite(LED_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
    digitalWrite(LED_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
}

// Status-LED: `blinks`-mal kurz aufleuchten lassen. Die Anzahl entspricht
// dem Verbindungszustand, den der CheckerTask ermittelt:
//   1x = WLAN wird gesucht
//   2x = TCP-Verbindung wird aufgebaut
//   3x = schlechtes Signal / Fehler
void showStatus(int blinks) {
    for (int i = 0; i < blinks; i++)
        blink(200);
}

// ------------------------- Ton (Piezo) ------------------------------------
void playTone() {
    // Ton nur abspielen, wenn der Kippschalter auf AUDIO steht.
    if (AUDIO_MODE == false) {
        stopTone();
        return;
    }
    // Ton nur anmachen, wenn ton noch nicht an ist. sonst knackt es
    if (soundIsOn == false) {
        tone(PIEZO_PIN, SOUND_FREQ);   // Rechtecksignal mit SOUND_FREQ Hz
        soundIsOn = true;       // sonst wackelt der ton
        LOGV(TAG, "turned Piezo on");
    }
}
void stopTone() {
    if (soundIsOn)
        noTone(PIEZO_PIN);   // PWM-Signal am Piezo-Pin anhalten
    soundIsOn = false;
    LOGV(TAG, "turned Piezo off");
}

// ------------------------- Kippschalter einlesen --------------------------
void checkPins() {
    // Jeder Kippschalter liegt mit Pull-up auf HIGH; bei aktivem Schalter
    // zieht der Kontakt auf LOW. digitalRead() liefert LOW (0) = aktiv, HIGH (1) = inaktiv
    AUDIO_MODE   = !digitalRead(AUDIO_MODE_PIN);
    PRINTER_MODE = !digitalRead(PRINTER_MODE_PIN);
    HOTSPOT_MODE = !digitalRead(HOTSPOT_MODE_PIN);
    TEST_MODE    = !digitalRead(TEST_MODE_PIN);

    if (!last_audio_mode && AUDIO_MODE)
        LOGI(TAG, "switched audio mode on");
    if (last_audio_mode && !AUDIO_MODE)
        LOGI(TAG, "switched audio mode off");
    if (!last_printer_mode && PRINTER_MODE)
        LOGI(TAG, "switched printer mode on");
    if (last_printer_mode && !PRINTER_MODE)
        LOGI(TAG, "switched printer mode off");
    if (!last_hotspot_mode && HOTSPOT_MODE)
        LOGI(TAG, "switched hotspot mode on");
    if (last_hotspot_mode && !HOTSPOT_MODE)
        LOGI(TAG, "switched hotspot mode off");
    if (!last_test_mode && TEST_MODE)
        LOGI(TAG, "switched test mode on");
    if (last_test_mode && !TEST_MODE)
        LOGI(TAG, "switched test mode off");

    last_audio_mode   = AUDIO_MODE;
    last_printer_mode = PRINTER_MODE;
    last_hotspot_mode = HOTSPOT_MODE;
    last_test_mode    = TEST_MODE;
}

// ------------------------- MOSFET-Test ------------------------------------
void testMosfet() {
    // MOSFET kurz aus- und wieder einschalten. Dient als Test der
    // Selbsthalte-Schaltung: Fällt die Versorgung weg, geht der ESP sauber
    // aus (weil der MOSFET den Supercap vom ESP32 trennt).
    digitalWrite(MOSFET_PIN, LOW);
    LOGV(TAG, "turned MOSFET off");
    vTaskDelay(pdMS_TO_TICKS(100));
    digitalWrite(MOSFET_PIN, HIGH);
    LOGV(TAG, "turned MOSFET on");
}

// ---------------------------------------------------------------------------
// button hat pull up und ist in gedrücktem zustand auf GND
bool buttonIsPressed() { return !digitalRead(BUTTON_PIN); }

void setLedHigh() { digitalWrite(LED_PIN, HIGH); }

void setLedLow() { digitalWrite(LED_PIN, LOW); }


// =============================================================================
// printer.cpp  –  Thermodrucker über UART1 (ESC/POS-Protokoll)
//
// Der Drucker wird mit rohen ESC/POS-Kommandos angesteuert:
//   ESC @   -> Drucker zurücksetzen
//   ESC 7   -> Heizparameter einstellen
//   ESC *   -> Grafikmodus starten
// Jede "Spalte" des Druckbilds besteht aus 8 vertikalen Punkten (1 Byte).
// =============================================================================

static void writeByte(uint8_t byte) { Serial1.write(byte); }  // ein einzelnes Byte zum Drucker

void printerInit() { 
    Serial1.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN); 
    LOGI(TAG, "Printer UART1 initialisiert (TX=%d RX=%d)", TX_PIN, RX_PIN);
}

void makePrinterReady() {
  // ESC @ : Drucker auf Werkszustand zurücksetzen.
  writeByte(27);   // ESC
  writeByte('@');  // @
  vTaskDelay(1);

  // ESC 7 n1 n2 n3 : Heizparameter konfigurieren.
  writeByte(27);   // ESC
  writeByte(55);   // 7
  writeByte(1);    // n1: Punkte, die gleichzeitig heizen dürfen
  writeByte(255);  // n2: Heizzeit (hoch = langsamer, aber dunkler)
  writeByte(255);  // n3: Pause zwischen Heizungen (hoch = weniger Strom)
  vTaskDelay(1);
}

void print(bool top_line[384], bool bottom_line[384]) {
  // Vor jeder Zeile warten, damit der Supercap zwischen zwei Zeilen nachgeladen
  // ist. Ohne das wäre der Abdruck zu blass.
  vTaskDelay(pdMS_TO_TICKS(PRINT_LINE_DELAY_MS));

  // ESC * 1 nL nH : Grafikmodus "8-dot double density" starten.
  writeByte(27);
  writeByte('*');
  writeByte((uint8_t)1);    // Mode 1 = 8 Punkte vertikal, doppelte Dichte
  writeByte((uint8_t)128);  // nL = 128 Bits
  writeByte((uint8_t)1);    // nH = 1 -> Zeilenlänge = nL + nH*256 = 384
  vTaskDelay(1);

  for (int i = 0; i < 384; i++) {
    uint8_t column = 0b00010000;  // feste Trennlinie zwischen den zwei Zeilen
    if (top_line[i])
      column = column | 0b11000000;  // Punkte für die obere Zeile
    if (bottom_line[i])
      column = column | 0b00000110;  // Punkte für die untere Zeile

    writeByte(column);  // die Spalte zum Drucker senden

    if (i % 16 == 0)  // alle 16 Spalten kurz "durchatmen" (gibt dem Watchdog Luft)
      vTaskDelay(1);
  }
  vTaskDelay(pdMS_TO_TICKS(50));
}

void printerWriteByte(uint8_t byte) { writeByte(byte); }

void printerFlush() { Serial1.flush(); }  // warten, bis der UART-Puffer leer ist

