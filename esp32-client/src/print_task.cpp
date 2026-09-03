#include "tasks.h"

#include <Arduino.h>
#include <cstring>

#include "app_state.h"
#include "package.h"
#include "printer.h"

void PrintTask(void* pvParameters) {
  Package package;
  static bool top_line[384] = { false };
  static bool bottom_line[384] = { false };
  int index = 0;
  bool writing_top_line = true;

  while (true) {
    // um Polling zu vermeiden
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // wenn ein/kein Paket in der Leitung ist
    if (uxQueueMessagesWaiting(printQueue) == 0)
      continue;

    // Paket auslesen
    getPackageFromQueue(printQueue, package);

    // überspringen, wenn "no printer" mode
    if (NO_PRINTER_MODE == true)
      continue;

    makePrinterReady();

    // Paket abarbeiten
    for (uint8_t signal : package.payload) {
      uint8_t mask = 0b10000000;

      for (int j = 0; j < 8; j++) {
        // ob das in die erste oder zweite Zeile des Druckers kommt
        if (writing_top_line)
          top_line[index] = (signal & mask);
        else
          bottom_line[index] = (signal & mask);

        mask >>= 1;
        index++;

        // wenn eine Zeile gedruckt werden kann
        if (index == 384) {
          if (writing_top_line == true) {
            writing_top_line = false;
            index = 0;
          } else {
            print(top_line, bottom_line);
            memset(top_line, 0, sizeof(top_line));
            memset(bottom_line, 0, sizeof(bottom_line));
            writing_top_line = true;
            index = 0;
          }
        }
      }
    }
    if (index != 0) {
      print(top_line, bottom_line);
      memset(top_line, 0, sizeof(top_line));
      memset(bottom_line, 0, sizeof(bottom_line));
      writing_top_line = true;
      index = 0;
    }
    // alle Zeilen flushen
    printer.flush();
    // Zeilenvorschub
    printer.write('\n');
    printer.write('\n');
    printer.flush();
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}
