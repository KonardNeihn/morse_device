#include "package.h"

#include <Arduino.h>

// ist nötig, um std::vector<> senden zu können (liegt im Heap)
bool putPackageIntoQueue(QueueHandle_t queue, const Package& package) {
  Package* copy = new Package(package);
  if (xQueueSend(queue, &copy, 0) != pdPASS) {
    Serial.printf("Queue overflow!!!\n");
    delete copy;
    return false;
  }
  return true;
}

// ist nötig, um std::vector<> senden zu können (liegt im Heap)
bool getPackageFromQueue(QueueHandle_t queue, Package& package) {
  Package* p;
  if (xQueueReceive(queue, &p, 0) != pdPASS)
    return false;

  package = *p;  // Deep Copy (std::vector kopiert korrekt)
  delete p;
  return true;
}

std::string packageToText(const Package& package) {
  std::string text = "";
  for (int i = 0; i < package.size; i++) {
    uint8_t mask = 0b10000000;
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
