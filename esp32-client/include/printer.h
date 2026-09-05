#pragma once

// =============================================================================
// printer.h  –  Ansteuerung des Thermodruckers (ESC/POS)
//
// Der Drucker hängt an UART2 (GPIO17 = TX, GPIO16 = RX) und wird über das
// ESC/POS-Protokoll angesprochen. Ein Morse-Paket wird in eine "Grafikzeile"
// aus zwei übereinanderliegenden Punktreihen übersetzt und gedruckt.
//
//   printerInit()      installiert den UART-Treiber
//   makePrinterReady() setzt den Drucker zurück + Heizparameter
//   print()            gibt eine Zeile (2 Punktreihen) aus
// =============================================================================

#include <cstdint>

void printerInit();
void makePrinterReady();
void print(bool top_line[384], bool bottom_line[384]);  // 384 Pixel breite Zeile
void printerWriteByte(uint8_t byte);                    // ein rohes Byte senden
void printerFlush();                                    // Ausgabepuffer leeren

