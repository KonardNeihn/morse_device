// =============================================================================
// package.cpp  –  Queue-Hilfsfunktionen für Package-Objekte
//
// Ein Package enthält einen std::vector (der liegt im Heap). Durch die Queue
// werden deshalb nur ZEIGER auf Package geschoben. Beim Einfügen wird eine
// Kopie auf dem Heap angelegt, beim Herausnehmen wieder freigegeben. So wird
// das eigentlich kopierbare std::vector-Objekt sicher zwischen Tasks geteilt.
// =============================================================================

#include "package.h"

#include <esp_log.h>

static const char* TAG = "package";

// Paket in die Queue legen (Zeiger auf eine Heap-Kopie).
bool putPackageIntoQueue(QueueHandle_t queue, const Package& package) {
  Package* copy = new Package(package);         // Kopie auf dem Heap anlegen
  // xQueueSend legt den Zeiger in die Queue; "0" = nicht warten, wenn voll.
  if (xQueueSend(queue, &copy, 0) != pdPASS) {
    ESP_LOGW(TAG, "Queue overflow!!!");         // Queue voll -> Kopie verwerfen
    delete copy;
    return false;
  }
  return true;
}

// Paket aus der Queue holen (Kopie herausnehmen und Heap wieder freigeben).
bool getPackageFromQueue(QueueHandle_t queue, Package& package) {
  Package* p;
  // xQueueReceive holt den Zeiger aus der Queue; "0" = nicht warten, wenn leer.
  if (xQueueReceive(queue, &p, 0) != pdPASS)
    return false;                             // nichts in der Queue

  package = *p;   // Deep Copy: der std::vector wird korrekt kopiert
  delete p;       // Heap-Objekt wieder freigeben
  return true;
}

// Paket als 0/1-String ausgeben (für Log-Ausgaben, z. B. "11110000...").
std::string packageToText(const Package& package) {
  std::string text = "";
  for (int i = 0; i < package.size; i++) {
    uint8_t mask = 0b10000000;  // mit dem höchstwertigen Bit (MSB) beginnen
    for (int j = 0; j < 8; j++) {
      if (package.payload[i] & mask)
        text += "1";
      else
        text += "0";
      mask >>= 1;
    }
  }
  return text;
}

