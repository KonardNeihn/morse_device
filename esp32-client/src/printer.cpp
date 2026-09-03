#include "printer.h"

#include <Arduino.h>

#include "app_state.h"

void makePrinterReady() {
  // Reset with ESC @
  printer.write(27);   // ESC
  printer.write('@');  // @
  vTaskDelay(1);

  // configure heating parameters ESC 7
  printer.write(27);   // ESC
  printer.write(55);   // 7
  printer.write(1);    // n1   → Heizpunkte, die gleichzeitig laufen dürfen
  printer.write(255);  // n2   → Länge Heizzeit (hoch -> langsam, aber dunkler)
  printer.write(255);  // n3   → Pause zwischen Heizungen (hoch -> Strom sinkt stark)
  vTaskDelay(1);
}

void print(bool top_line[384], bool bottom_line[384]) {
  // Größe des Bildes
  printer.write(27);
  printer.write('*');
  printer.write((uint8_t)1);    // Mode 1 = 8-dot double density (384 Pixel)
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

    if (i % 16 == 0)  // alle 16 Spalten mal kurz durchatmen (evtl. auch wichtig für den Watchdog)
      vTaskDelay(1);
  }
  vTaskDelay(50 / portTICK_PERIOD_MS);
}
