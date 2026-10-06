// =============================================================================
// print_task.cpp  –  "Print Task" (Priorität 2)
//
// Nimmt Morse-Pakete aus der printQueue (vom PlaybackTask) und bereitet sie
// für den Thermodrucker auf. Der Drucker druckt eine Zeile aus zwei
// übereinanderliegenden Punktreihen (384 Pixel breit). Deshalb werden die
// Bits abwechselnd in eine obere (top_line) und untere (bottom_line) Reihe
// einsortiert, bis 384 Pixel voll sind, und dann gedruckt.
// =============================================================================

#include "tasks.h"

#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "app_state.h"
#include "config.h"
#include "network.h"
#include "package.h"
#include "printer.h"

void PrintTask(void* pvParameters) {
  Package package;
  static bool top_line[384] = { false };    // obere Punktreihe (384 Pixel)
  static bool bottom_line[384] = { false }; // untere Punktreihe (384 Pixel)
  int index = 0;                            // Position in der aktuellen Reihe
  bool writing_top_line = true;             // zuerst in die obere Reihe schreiben

  // Erstaufladung des Supercaps abwarten, bevor überhaupt gedruckt werden darf
  // (~30 s nach dem Booten). Zwischen den Zeilen wartet print() dann nur noch 5 s.
  vTaskDelay(pdMS_TO_TICKS(PRINT_BOOT_DELAY_MS));

  while (true) {
    vTaskDelay(pdMS_TO_TICKS(100));  // kleine Pause (Polling vermeiden)

    // Kein Paket da? -> weiter warten.
    if (uxQueueMessagesWaiting(printQueue) == 0)
      continue;

    // Paket auslesen.
    getPackageFromQueue(printQueue, package);

    // Im "ohne Drucker"-Modus das Paket verwerfen.
    if (NO_PRINTER_MODE == true)
      continue;

    makePrinterReady();  // Drucker zurücksetzen + Heizparameter

    // Paket Bit für Bit in die zwei Punktreihen einsortieren.
    for (uint8_t signal : package.payload) {
      uint8_t mask = 0b10000000;

      for (int j = 0; j < 8; j++) {
        // Abwechselnd in obere/untere Reihe schreiben.
        if (writing_top_line)
          top_line[index] = (signal & mask);
        else
          bottom_line[index] = (signal & mask);

        mask >>= 1;
        index++;

        // 384 Pixel = eine volle Druckzeile erreicht.
        if (index == 384) {
          if (writing_top_line == true) {
            // Obere Reihe voll -> jetzt die untere füllen.
            writing_top_line = false;
            index = 0;
          } else {
            // Beide Reihen voll -> drucken und von vorn beginnen.
            print(top_line, bottom_line);
            memset(top_line, 0, sizeof(top_line));    // Reihe mit Nullen füllen (= leer)
            memset(bottom_line, 0, sizeof(bottom_line));
            writing_top_line = true;
            index = 0;
          }
        }
      }
    }
    // Rest (weniger als 384 Pixel) trotzdem drucken.
    if (index != 0) {
      print(top_line, bottom_line);
      memset(top_line, 0, sizeof(top_line));
      memset(bottom_line, 0, sizeof(bottom_line));
      writing_top_line = true;
      index = 0;
    }
    // Puffer leeren und zwei Zeilen Vorschub für Abstand.
    printerFlush();
    printerWriteByte('\n');
    printerWriteByte('\n');
    printerFlush();
    vTaskDelay(pdMS_TO_TICKS(50));

    // Druck ist fertig -> jetzt erst das Zustell-ACK an den Server schicken.
    // (Nur für Pakete, die vom Server kamen: status 1 = Nachricht, 3 = Check.)
    if (package.status == 1 || package.status == 3)
      queueDeliveryAck(package.msg_id);
  }
}

