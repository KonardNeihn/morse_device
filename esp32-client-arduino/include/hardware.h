#pragma once

// =============================================================================
// hardware.h  –  Zugriff auf die angeschlossene Hardware
//
// Bündelt alles, was direkt mit den Pins zu tun hat:
// GPIOs (Morse-Taste, Kippschalter, Status-LED, MOSFET) und die Ton-Ausgabe
//
// Arduino-typisch: GPIOs über pinMode()/digitalWrite()/digitalRead(),
// Lautsprecher-Ton über die Standard-Funktionen tone()/noTone().
// =============================================================================

// Initialisiert alle GPIOs als Ein- oder Ausgänge und schaltet den MOSFET ein.
// Wird einmal beim Start in setup() aufgerufen.
void hardwareInit();

// Status-LED: `blinks`-mal kurz aufleuchten lassen. Die Anzahl entspricht dem
// Verbindungszustand des Checker-Tasks (1x = WLAN suchen, 2x = TCP, 3x =
// schlechtes Signal).
void showStatus(int blinks);

// Ton an/aus über den Piezo-Lautsprecher.
// playTone() prüft selbst, ob AUDIO_MODE aktiv ist, und gibt nur dann Ton aus.
// stopTone() schaltet den Ton unabhängig vom Modus aus.
void playTone();
void stopTone();

// Liest alle vier Kippschalter digital ein und setzt die globalen Modus-Flags
// (AUDIO_MODE, PRINTER_MODE, HOTSPOT_MODE, TEST_MODE) in app.h.
void checkPins();

// MOSFET kurz aus- und wieder einschalten. Dient als Test der Selbsthalte-
// Schaltung: Fällt die Versorgung weg, geht der ESP sauber aus (der MOSFET
// trennt den Supercap vom ESP32).
void testMosfet();

bool buttonIsPressed();

void setLedHigh();

void setLedLow();

// =============================================================================
// Thermodrucker (ESC/POS) – ehemals eigene Datei printer.h
//
// Der Drucker hängt an UART1 (GPIO4 = TX, GPIO5 = RX) und wird über das
// ESC/POS-Protokoll angesprochen. Ein Morse-Paket wird in eine "Grafikzeile"
// aus zwei übereinanderliegenden Punktreihen übersetzt und gedruckt.
//
//   printerInit()      initialisiert Serial1 (UART1)
//   makePrinterReady() setzt den Drucker zurück + Heizparameter
//   print()            gibt eine Zeile (2 Punktreihen) aus
// =============================================================================

#include <cstdint>

void printerInit();
void makePrinterReady();
void print(bool top_line[384], bool bottom_line[384]);  // 384 Pixel breite Zeile
void printerWriteByte(uint8_t byte);                    // ein rohes Byte senden
void printerFlush();                                    // Ausgabepuffer leeren

